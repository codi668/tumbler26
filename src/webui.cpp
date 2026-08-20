#include "webui.h"
#include "config.h"
#include "state.h"

#include "encoder.h"
#include "autotune.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

namespace {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Eine einzige Seite, komplett im Flash - kein separates Dateisystem noetig.
const char PAGE_HTML[] = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tumbler</title>
<style>
:root{--bg:#111418;--card:#1b1f26;--line:#2a3038;--fg:#e8ecf1;--dim:#8b95a3;
  --ok:#2ecc71;--warn:#e0a020;--bad:#e04848;--acc:#4a9eda}
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--fg);
  margin:0;padding:12px;max-width:640px;margin:0 auto}
header{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}
h1{font-size:1.1em;margin:0;font-weight:600}
#conn{font-size:.75em;color:var(--dim);display:flex;align-items:center;gap:6px}
#dot{width:8px;height:8px;border-radius:50%;background:var(--bad)}
#dot.up{background:var(--ok)}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;
  padding:12px;margin-bottom:12px}
.top{display:flex;align-items:center;gap:16px}
#gauge{flex:0 0 84px}
.readout{flex:1}
#state{font-size:.7em;letter-spacing:.08em;padding:3px 9px;border-radius:4px;
  display:inline-block;background:#333;color:#ccc;font-weight:600}
#state.run{background:var(--ok);color:#07230f}
#state.wait{background:var(--warn);color:#241800}
#state.fall{background:var(--bad);color:#fff}
#angle{font-size:2.1em;font-weight:700;line-height:1.1;margin:4px 0 2px;
  font-variant-numeric:tabular-nums}
.sub{font-size:.8em;color:var(--dim);font-variant-numeric:tabular-nums}
canvas{width:100%;display:block;border-radius:6px}
.legend{display:flex;gap:14px;font-size:.72em;color:var(--dim);margin-top:6px}
.legend i{display:inline-block;width:9px;height:3px;border-radius:2px;
  margin-right:4px;vertical-align:middle}
.row{display:flex;gap:8px;margin-bottom:12px}
button{flex:1;padding:13px 4px;font-size:.92em;font-weight:600;border:0;
  border-radius:8px;background:var(--ok);color:#07230f;cursor:pointer}
button.stop{background:var(--bad);color:#fff}
button.alt{background:#39414d;color:var(--fg)}
h2{font-size:.75em;text-transform:uppercase;letter-spacing:.08em;
  color:var(--dim);margin:0 0 10px;font-weight:600}
.p{margin-bottom:11px}
.plabel{display:flex;justify-content:space-between;font-size:.85em;margin-bottom:4px}
.pval{color:var(--acc);font-variant-numeric:tabular-nums;font-weight:600}
input[type=range]{width:100%;accent-color:var(--acc);margin:0}
.toggle{display:flex;gap:8px}
.toggle button{padding:8px;font-size:.85em;background:#39414d;color:var(--dim)}
.toggle button.on{background:var(--acc);color:#04141f}
#toast{position:fixed;left:50%;transform:translateX(-50%);bottom:20px;
  background:var(--acc);color:#04141f;padding:9px 18px;border-radius:8px;
  font-size:.85em;font-weight:600;opacity:0;transition:opacity .25s;
  pointer-events:none}
#toast.show{opacity:1}
</style></head><body>

<header>
  <h1>Tumbler V2</h1>
  <div id="conn"><span id="dot"></span><span id="connt">verbinde...</span></div>
</header>

<div class="card">
  <div class="top">
    <svg id="gauge" viewBox="0 0 100 100" width="84" height="84">
      <circle cx="50" cy="50" r="46" fill="none" stroke="#2a3038" stroke-width="2"/>
      <line x1="50" y1="50" x2="50" y2="96" stroke="#2a3038" stroke-width="2"/>
      <g id="bot"><rect x="44" y="14" width="12" height="38" rx="3" fill="#4a9eda"/>
        <circle cx="50" cy="58" r="9" fill="none" stroke="#8b95a3" stroke-width="3"/></g>
    </svg>
    <div class="readout">
      <span id="state">--</span>
      <div id="angle">--.--&deg;</div>
      <div class="sub" id="sub">Richtung --&deg; &middot; PWM -- &middot; Lenkung --</div>
    </div>
  </div>
</div>

<div class="card">
  <canvas id="chart" height="150"></canvas>
  <div class="legend">
    <span><i style="background:#2ecc71"></i>Winkelfehler &plusmn;10&deg;</span>
    <span><i style="background:#4a9eda"></i>PWM &plusmn;255</span>
    <span><i style="background:#e0a020"></i>Lenkung &plusmn;60</span>
    <span><i style="background:#b06ce0"></i>Weg &plusmn;2000</span>
  </div>
</div>

<div class="row">
  <button onclick="cmd('START')">START</button>
  <button class="stop" onclick="cmd('STOP')">STOP</button>
  <button class="alt" onclick="cmd('ZERO')">ZERO</button>
  <button class="alt" onclick="cmd('HOME')">HOME</button>
  <button class="alt" onclick="cmd('SAVE')">SAVE</button>
</div>

<div class="row">
  <button class="alt" id="tuneBtn" onclick="cmd('TUNE')">AUTO-ABSTIMMUNG</button>
</div>
<div class="card" id="tuneBox" style="display:none">
  <h2>Abstimmung laeuft</h2>
  <div class="sub" id="tuneInfo">--</div>
  <div class="sub" style="margin-top:6px">
    Er muss dabei durchgehend balancieren &mdash; nach einem Sturz bitte wieder
    aufstellen, dann macht er von selbst weiter.
  </div>
</div>

<div class="card"><h2>Balance</h2><div id="g1"></div></div>
<div class="card"><h2>Richtung halten</h2><div id="g2"></div></div>
<div class="card"><h2>Position halten</h2><div id="g3"></div></div>
<div class="card"><h2>Fahren</h2>
  <div class="p">
    <div class="plabel"><span>Tempo</span><span class="pval" id="vspd">1500</span></div>
    <input type="range" id="spd" min="200" max="4000" step="100" value="1500">
  </div>
  <div class="row" style="margin-bottom:8px">
    <button class="alt" onclick="drive(-1)">&#9660; ZUR&Uuml;CK</button>
    <button class="stop" onclick="cmd('HALT')">HALT</button>
    <button class="alt" onclick="drive(1)">VOR &#9650;</button>
  </div>
  <div class="row" style="margin-bottom:0">
    <button class="alt" onclick="cmd('TURN=-45')">&#8634; LINKS</button>
    <button class="alt" onclick="cmd('TURN=0')">GERADE</button>
    <button class="alt" onclick="cmd('TURN=45')">RECHTS &#8635;</button>
  </div>
  <div class="sub" id="driveInfo" style="margin-top:8px">steht</div>
</div>

<div class="card"><h2>Aufschwingen</h2><div id="g4"></div>
  <div class="sub">Steht er beim START schraeg, faehrt er sich selbst hoch.
    UPPWM ist der Schwung, UPMAX der groesste Winkel, aus dem er es versucht
    &mdash; UPPWM&nbsp;=&nbsp;0 schaltet es ab.</div>
</div>

<div class="card"><h2>Fahrregler</h2><div id="g5"></div></div>

<div id="toast"></div>

<script>
// [id, kommando, minimum, maximum, schrittweite, nachkommastellen, gruppe]
const P = [
  ['Kp','P',0,60,0.5,1,'g1'], ['Ki','I',0,2,0.01,2,'g1'],
  ['Kd','D',0,3,0.01,2,'g1'], ['minPwm','MINPWM',0,120,1,0,'g1'],
  ['trim','TRIM',-10,10,0.05,2,'g1'],
  ['Ykp','YP',0,15,0.1,1,'g2'], ['Ykd','YD',0,2,0.01,2,'g2'],
  ['Vkp','VP',0,0.01,0.0002,4,'g3'], ['Vki','VI',0,0.004,0.0001,4,'g3'],
  ['upPwm','UPPWM',0,255,5,0,'g4'], ['upMax','UPMAX',0,80,1,0,'g4'],
  ['Dkp','DP',0,0.01,0.0002,4,'g5'], ['Dki','DI',0,0.004,0.0001,4,'g5']
];
const SIGNS = [['sign','SIGN','Motorrichtung','g1'],
               ['ysign','YSIGN','Drehrichtung','g2']];

for (const [id,c,mn,mx,st,dec,grp] of P) {
  const d = document.createElement('div');
  d.className = 'p';
  d.innerHTML = '<div class="plabel"><span>'+id+'</span>'+
    '<span class="pval" id="v'+id+'">--</span></div>'+
    '<input type="range" id="'+id+'" min="'+mn+'" max="'+mx+'" step="'+st+'">';
  document.getElementById(grp).appendChild(d);
  const s = document.getElementById(id);
  // input = waehrend des Ziehens anzeigen, change = erst beim Loslassen senden,
  // sonst flutet ein Schieberegler den WebSocket mit hunderten Kommandos.
  s.addEventListener('input', e => {
    document.getElementById('v'+id).textContent = (+e.target.value).toFixed(dec);
  });
  s.addEventListener('change', e => cmd(c+'='+e.target.value));
}
for (const [id,c,label,grp] of SIGNS) {
  const d = document.createElement('div');
  d.className = 'p';
  d.innerHTML = '<div class="plabel"><span>'+label+'</span></div>'+
    '<div class="toggle"><button id="'+id+'p">+1</button>'+
    '<button id="'+id+'m">-1</button></div>';
  document.getElementById(grp).appendChild(d);
  document.getElementById(id+'p').onclick = () => cmd(c+'=1');
  document.getElementById(id+'m').onclick = () => cmd(c+'=-1');
}

// --- Verlauf ---------------------------------------------------------------
const N = 240;                       // ~24 s bei 10 Hz
const hist = {e:[], p:[], s:[], d:[]};
const cv = document.getElementById('chart'), cx = cv.getContext('2d');
function push(a,v){ a.push(v); if (a.length > N) a.shift(); }
function draw() {
  const w = cv.width = cv.clientWidth * devicePixelRatio;
  const h = cv.height = 150 * devicePixelRatio;
  cx.clearRect(0,0,w,h);
  cx.strokeStyle = '#2a3038'; cx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {          // Raster
    const y = h * i / 4;
    cx.globalAlpha = i === 2 ? 1 : .45;   // Mittellinie = Null
    cx.beginPath(); cx.moveTo(0,y); cx.lineTo(w,y); cx.stroke();
  }
  cx.globalAlpha = 1; cx.lineWidth = 1.6 * devicePixelRatio;
  const trace = (arr, scale, col) => {
    if (arr.length < 2) return;
    cx.strokeStyle = col; cx.beginPath();
    arr.forEach((v,i) => {
      const x = w * i / (N-1);
      const y = h/2 - Math.max(-1, Math.min(1, v/scale)) * h/2 * .92;
      i ? cx.lineTo(x,y) : cx.moveTo(x,y);
    });
    cx.stroke();
  };
  trace(hist.p, 255,  '#4a9eda');
  trace(hist.s, 60,   '#e0a020');
  trace(hist.d, 2000, '#b06ce0');
  trace(hist.e, 10,   '#2ecc71');
}
addEventListener('resize', draw);

// --- Verbindung ------------------------------------------------------------
let ws, toastT;
function toast(m) {
  const t = document.getElementById('toast');
  t.textContent = m; t.className = 'show';
  clearTimeout(toastT); toastT = setTimeout(() => t.className = '', 1800);
}
function cmd(c) { if (ws && ws.readyState === 1) ws.send(c); }
const spd = document.getElementById('spd');
spd.addEventListener('input', e => {
  document.getElementById('vspd').textContent = e.target.value;
});
function drive(richtung) { cmd('FWD=' + richtung * spd.value); }
function setConn(up) {
  document.getElementById('dot').className = up ? 'up' : '';
  document.getElementById('connt').textContent = up ? 'verbunden' : 'getrennt';
}
function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 1000); };
  ws.onmessage = e => {
    const p = e.data.split(',');
    if (p[0] === 'M') { toast(p[1]); return; }
    if (p[0] !== 'T') return;
    // T,winkel,rate,pwm,laeuft,angefordert,sturz,Kp,Ki,Kd,minPwm,trim,sign,
    //   gier,lenkung,Ykp,Ykd,ysign
    const ang = +p[1], pwm = +p[3], yaw = +p[13], steer = +p[14], trim = +p[11];
    const pos = +p[18], bias = +p[20];
    const run = p[4] === '1', req = p[5] === '1', fall = p[6] === '1';

    document.getElementById('angle').textContent = ang.toFixed(2) + '°';
    document.getElementById('sub').textContent =
      'Richtung ' + yaw.toFixed(1) + '° · PWM ' + pwm +
      ' · Lenkung ' + steer + ' · Weg ' + pos + ' · Soll ' + bias.toFixed(1) + '°';
    document.getElementById('bot').setAttribute(
      'transform', 'rotate(' + (-ang).toFixed(1) + ' 50 58)');

    // Abstimmung: Felder 23..27
    const tuning = p[23] === '1';
    const box = document.getElementById('tuneBox');
    const btn = document.getElementById('tuneBtn');
    box.style.display = tuning ? '' : 'none';
    btn.textContent = tuning ? 'ABSTIMMUNG BEENDEN' : 'AUTO-ABSTIMMUNG';
    btn.className = tuning ? 'stop' : 'alt';
    if (tuning) {
      const namen = ['Balance','minPwm','Richtung','Position','Feinschliff'];
      document.getElementById('tuneInfo').textContent =
        'Stufe ' + (+p[24]+1) + '/5 ' + (namen[+p[24]] || '') +
        ' \u00b7 ' + p[27] + ' \u00b7 ' + p[25] + ' Versuche \u00b7 bester Wert ' +
        (+p[26]).toFixed(2);
    }

    // Fahrzustand: Felder 31..33
    const soll = +p[32], ist = +p[31], dreh = +p[33];
    document.getElementById('driveInfo').textContent =
      (Math.abs(soll) < 1 && Math.abs(ist) < 1)
        ? 'steht'
        : 'Soll ' + soll + ' \u00b7 ist ' + ist + ' Impulse/s' +
          (Math.abs(dreh) > 0.5 ? ' \u00b7 Drehung ' + dreh + '\u00b0/s' : ' \u00b7 geradeaus');

    const st = document.getElementById('state');
    if (p[28] === '1') { st.textContent = 'AUFSTEHEN';  st.className = 'wait'; }
    else if (fall && !run)  { st.textContent = 'UMGEFALLEN'; st.className = 'fall'; }
    else if (run)      { st.textContent = 'BALANCIERT'; st.className = 'run'; }
    else if (req)      { st.textContent = 'WARTET';     st.className = 'wait'; }
    else               { st.textContent = 'AUS';        st.className = ''; }

    push(hist.e, ang - trim - bias); push(hist.p, pwm);
    push(hist.s, steer); push(hist.d, pos);
    draw();

    // Regler nur nachfuehren, solange niemand gerade daran zieht.
    const vals = {Kp:p[7],Ki:p[8],Kd:p[9],minPwm:p[10],trim:p[11],
                  Ykp:p[15],Ykd:p[16],Vkp:p[21],Vki:p[22],
                  upPwm:p[29],upMax:p[30],Dkp:p[34],Dki:p[35]};
    for (const [id,,,,,dec] of P) {
      const el = document.getElementById(id);
      if (el !== document.activeElement) {
        el.value = vals[id];
        document.getElementById('v'+id).textContent = (+vals[id]).toFixed(dec);
      }
    }
    for (const [id] of SIGNS) {
      const v = id === 'sign' ? +p[12] : +p[17];
      document.getElementById(id+'p').className = v > 0 ? 'on' : '';
      document.getElementById(id+'m').className = v < 0 ? 'on' : '';
    }
  };
}
connect();
</script>
</body></html>
)HTML";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type != WS_EVT_DATA) return;
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (!info->final || info->index != 0 || info->len != len) return; // nur kurze Einzelframes
    if (info->opcode != WS_TEXT) return;

    String msg;
    msg.reserve(len);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    handleCommand(msg);
}

} // namespace

void webuiBegin()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print(F("WLAN verbinde"));
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000)
    {
        delay(250);
        Serial.print('.');
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print(F("\nWLAN verbunden, IP="));
        Serial.println(WiFi.localIP());
    }
    else
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.print(F("\nKein WLAN gefunden, Accesspoint '"));
        Serial.print(AP_SSID);
        Serial.print(F("' aktiv, IP="));
        Serial.println(WiFi.softAPIP());
    }

    MDNS.begin(HOSTNAME);
    MDNS.addService("http", "tcp", 80);

    // Update ueber WLAN. Beim Balancieren faellt das USB-Kabel staendig aus
    // bzw. steht im Weg - ohne OTA muesste man den Roboter zum Flashen jedes
    // Mal anleinen. Waehrend des Uploads sind die Motoren aus.
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.onStart([]() {
        running = false;
        requested = false;
        digitalWrite(PIN_STBY, LOW);   // Endstufen aus, bevor der Flash beschrieben wird
        Serial.println(F("OTA-Update startet"));
    });
    ArduinoOTA.begin();

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", PAGE_HTML);
    });
    server.begin();
}

void webuiLoop()
{
    ArduinoOTA.handle();

    static uint32_t lastCleanupMs = 0;
    uint32_t now = millis();
    if (now - lastCleanupMs >= 1000)   // tote Clients regelmaessig aufraeumen
    {
        lastCleanupMs = now;
        ws.cleanupClients();
    }
}

void webuiNotify(const char *msg)
{
    if (ws.count() == 0) return;
    char buf[80];
    snprintf(buf, sizeof(buf), "M,%s", msg);
    ws.textAll(buf);
}

void webuiSendTelemetry()
{
    if (ws.count() == 0) return;

    char buf[460];
    snprintf(buf, sizeof(buf),
             "T,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.3f,%.2f,%.1f,%.2f,%.0f,%.2f,%d,%.2f,%.2f,%.0f"
             ",%ld,%.0f,%.2f,%.4f,%.4f,%d,%d,%d,%.2f,%s,%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.4f,%.4f",
             angleDeg, gyroRateDs, lastPwmOut,
             running ? 1 : 0, requested ? 1 : 0, fallFlag ? 1 : 0,
             Kp, Ki, Kd, minPwm, trim, outSign,
             yawDeg, (int)lastSteer, Ykp, Ykd, yawSign,
             encoderPos() - posTarget, wheelSpeed, tiltBias, Vkp, Vki,
             autotuneActive() ? 1 : 0, autotuneStage(), autotuneTrial(),
             autotuneBestCost(), autotunePhase(),
             swingActive ? 1 : 0, upPwm, upMax,
             driveSpeed, driveWish, turnRate, Dkp, Dki);
    ws.textAll(buf);
}
