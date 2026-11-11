//Clément LE PRIOL
//Martin Magnum 850 DMX / sACN / Art-Net controller
//Github : https://github.com/Peemouse/Martin-Magnum-DMX-controller
//
// Requires : Arduino-ESP32 core 3.x, esp_dmx, Adafruit SSD1306 + GFX, ArduinoJson 7,
//            ESP32Async/ESPAsyncWebServer + ESP32Async/AsyncTCP.
// Do NOT include the WIZnet "Ethernet" library: Ethernet is handled by ETH.h (lwIP).

#define FW_MAJOR      1
#define FW_MINOR      0

#define DEBUG

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "HW.h"
#include "parameters.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ETH.h>
#include <esp_dmx.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "ESPAsyncE131ETH.h"
#include "ArtNetRx.h"

// Defaults (can be overridden in parameters.h)
#ifndef AP_SSID
#define AP_SSID             "Magnum850"      // Wi-Fi access point name
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD         "fogmachine"     // WPA2 password, 8 characters minimum : CHANGE IT
#endif
#ifndef AP_CHANNEL
#define AP_CHANNEL          6
#endif
#ifndef AP_MAX_CLIENTS
#define AP_MAX_CLIENTS      4
#endif
#ifndef WEB_FOG_TIMEOUT_MS
#define WEB_FOG_TIMEOUT_MS  700              // dead-man: no "hold to fog" heartbeat for this long => fog stops
#endif
#ifndef WEB_FOG_MAX_MS
#define WEB_FOG_MAX_MS      10000            // a single web "hold to fog" can never last longer than this
#endif

struct RepeatBtn { bool held = false; unsigned long t0 = 0; unsigned long tLast = 0; };

//=============================================================================
// Settings (persisted in NVS)
//=============================================================================
uint16_t DMXstartAddress = 1;
uint16_t sacnUniverse    = UNIVERSE;
uint16_t artnetUniverse  = ARTNET_DEFAULT_UNIVERSE;
bool     wifiApEnabled   = false;     // Wi-Fi access point requested by the user (menu)

const uint8_t DEF_IP[4]   = {10, 0, 0, 1};
const uint8_t DEF_MASK[4] = {255, 255, 255, 0};
const uint8_t DEF_GW[4]   = {10, 0, 0, 250};
uint8_t cfgIp[4]   = {10, 0, 0, 1};
uint8_t cfgMask[4] = {255, 255, 255, 0};
uint8_t cfgGw[4]   = {10, 0, 0, 250};

// Network settings as applied at boot (a change needs a reboot)
uint8_t  bootIp[4], bootMask[4], bootGw[4];
uint16_t bootSacnUniverse;

Preferences preferences;

//=============================================================================
// DMX / network sources
//=============================================================================
dmx_port_t dmxPort = 1; //Serial port for DMX
byte DMXdata[DMX_PACKET_SIZE];            // wired DMX   (index 0 = start code)
uint8_t sacnData[DMX_PACKET_SIZE];        // sACN        (index 0 = start code)
uint8_t artnetData[DMX_PACKET_SIZE];      // Art-Net     (index 0 = start code)

enum : uint8_t { SRC_NONE = 0, SRC_DMX, SRC_SACN, SRC_ARTNET };
volatile uint8_t  activeSource = SRC_NONE;
volatile bool     dmxIsConnected = false;   // wired DMX
volatile uint32_t lastSacnMs = 0, lastArtnetMs = 0;

const char *sourceName(uint8_t s) {
  switch (s) {
    case SRC_DMX:    return "DMX";
    case SRC_SACN:   return "sACN";
    case SRC_ARTNET: return "ArtNet";
    default:         return "none";
  }
}

//=============================================================================
// Network objects
//=============================================================================
AsyncWebServer server(80);
AsyncEventSource events("/events");
ESPAsyncE131ETH e131;
ArtNetRx artnet;
bool networkStarted = false;
bool wifiApActive = false;                    // access point really running
volatile bool wifiApApplyRequested = false;

// Web "hold to fog" dead-man switch (see /api/fog)
volatile uint32_t webFogUntil = 0;            // fog allowed until this time, refreshed by the page heartbeat
volatile uint32_t webFogStart = 0;            // start of the current press
volatile uint32_t webFogStopMs = 0;           // time of the last explicit release

volatile bool rebootRequested = false;
volatile uint32_t rebootAt = 0;

// Firmware update (OTA over Ethernet) state
volatile bool     otaInProgress = false;     // true from the first chunk until reboot / failure
volatile bool     otaFailed = false;
volatile uint8_t  otaPercent = 0;
volatile uint32_t otaTotal = 0;              // firmware size in bytes (0 = unknown)
volatile uint32_t otaLastChunkMs = 0;
const char *volatile otaErrorMsg = "";
const void *volatile otaRejectedReq = nullptr;   // upload request refused (console active / busy)
const char *volatile otaRejectMsg = "";

//=============================================================================
// Display / UI / machine state
//=============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool btnEnterMemo = false, btnEnterLongPressMemo = false;
bool btnMinusShortPress = false, btnEnterShortPress = false, btnPlusShortPress = false;
bool btnEnterLongPress = false;
unsigned long btnEnterTimer = 0;
unsigned long inMenuTimer = 0;

volatile bool foggerHeating = false, foggerReady = false;
bool PWMactivated = true;
volatile bool forceFullFog = false;
volatile bool isCoolingDown = false;
bool fanRelay = false;

bool flag1s = true;

enum MenuState : uint8_t { MENU_HOME, MENU_MAIN, MENU_EDIT };
volatile MenuState menuState = MENU_HOME;

enum MenuItem : uint8_t { IT_FOG, IT_TON, IT_TOFF, IT_FAN, IT_DMX, IT_SACN, IT_ARTNET, IT_IP, IT_MASK, IT_GW, IT_WIFI, IT_COUNT };
const char *const ITEM_LABELS[IT_COUNT] = {"Fog", "TOn", "TOff", "Fan", "DMX", "sACN", "ArtN", "IP", "Mask", "GW", "WiFi"};

volatile uint8_t menuIndex = 0;   // selected item
volatile uint8_t editIndex = 0;   // selected octet when editing an IP

uint8_t fogIntensity = 0;
uint8_t tOn = 0;
uint8_t tOff = 0;

unsigned long fogOnTimer = 0;
unsigned long fogOffTimer = 0;

// Temporary full-screen message
volatile uint32_t msgUntil = 0;
const char *volatile msgText = "";

//=============================================================================
// Web page (served from flash, updated live through Server-Sent Events)
//=============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Martin Magnum 850 Monitor</title>
<style>
[hidden]{display:none!important}
body{font-family:Arial,sans-serif;text-align:center;margin:30px auto;max-width:480px;padding:0 12px}
.status{font-size:2em;margin:14px}
.ready_on{color:green;font-weight:bold}
.ready_off{color:gray}
.heating_on{color:red;font-weight:bold}
.heating_off{color:gray}
.fan_on{color:blue;font-weight:bold}
.fan_off{color:gray}
.unknown{color:gray;font-weight:normal}
table{margin:20px auto;border-collapse:collapse;text-align:left}
td{padding:4px 14px;border-bottom:1px solid #ddd}
.panel{margin:24px auto;padding:12px 16px;border:1px solid #ddd;border-radius:8px;text-align:left}
.panel h2{margin:0 0 10px;font-size:1.1em;text-align:center}
.row{display:flex;align-items:center;gap:10px;margin:10px 0}
.row label{width:110px}
.row input[type=range]{flex:1}
.row input[type=number]{width:80px;font-size:1em}
.row span{width:52px;text-align:right}
.small{font-size:.85em;color:gray;margin-top:6px}
#psht{display:block;width:100%;margin-top:14px;padding:18px;font-size:1.3em;border:2px solid #c60;border-radius:10px;background:#fff3e6;color:#c60;touch-action:none;user-select:none;-webkit-user-select:none;cursor:pointer}
#psht.on{background:#c60;color:#fff}
#psht:disabled{opacity:.4;cursor:not-allowed}
#stop{display:block;width:100%;margin-top:14px;padding:10px;font-size:1.1em;border:2px solid #c00;border-radius:10px;background:#fff;color:#c00;cursor:pointer}
#stop:active{background:#c00;color:#fff}
#conn{font-size:.9em;color:gray}
#upd{display:inline-block;margin-top:10px;font-size:.8em;color:#888;text-decoration:none;border:1px solid #ccc;border-radius:4px;padding:2px 10px}
#upd:hover{color:#000;border-color:#888}
</style>
</head>
<body>
<h1>Fog Machine Status</h1>
<div id="ready" class="status">READY ?</div>
<div id="heating" class="status">HEATING ?</div>
<div id="fan" class="status">FAN ?</div>
<table>
<tr><td>Control source</td><td id="source">-</td></tr>
<tr><td>Fog</td><td id="fog">-</td></tr>
<tr><td>Timer ON / OFF</td><td id="timers">-</td></tr>
<tr><td>Cooling mode</td><td id="cooling">-</td></tr>
<tr><td>DMX address</td><td id="dmx">-</td></tr>
<tr><td>sACN universe</td><td id="sacn">-</td></tr>
<tr><td>Art-Net universe</td><td id="artnet">-</td></tr>
<tr><td>Wi-Fi access point</td><td id="ap">-</td></tr>
<tr><td>IP (Ethernet)</td><td id="ip">-</td></tr>
<tr><td>Firmware</td><td id="fw">-</td></tr>
</table>

<div id="manual" class="panel" hidden>
<h2>Manual control</h2>
<div class="row"><label for="mFog">Fog</label><input type="range" id="mFog" min="0" max="100" value="0"><span id="mFogV">0 %</span></div>
<div class="row"><label for="mOn">Timer ON (s)</label><input type="number" id="mOn" min="0" max="255" value="0"></div>
<div class="row"><label for="mOff">Timer OFF (s)</label><input type="number" id="mOff" min="0" max="255" value="0"></div>
<div class="row"><label for="mFan">Fan relay</label><input type="checkbox" id="mFan"></div>
<div class="small">A timer set to 0 means continuous fog. Only available while no console controls the machine.</div>
<button id="stop" type="button">STOP (fog, timers = 0)</button>
<button id="psht" type="button" disabled>Hold to fog</button>
<div class="small">Fog runs only while the button is held. Releasing it, or losing the connection, stops the fog immediately.</div>
</div>

<div id="conn">connecting...</div>
<a id="upd" href="/update" hidden>Update FW</a>
<script>
const $ = id => document.getElementById(id);
const TIMEOUT_MS = 5000;   // the ESP32 pushes a message at least every 2 s
const psht = $('psht');
let src = null;
let lastMsg = Date.now();  // grace period at page load
let lastRetry = 0;
let lost = true;
let editHold = 0;          // do not overwrite the controls while the user is editing them
let fogTimer = null;       // heartbeat of the "hold to fog" button

function setUnknown() {
  $('upd').hidden = true;
  $('manual').hidden = true;
  fogStop(false);
  $('ready').textContent = 'READY ?';     $('ready').className = 'status unknown';
  $('heating').textContent = 'HEATING ?'; $('heating').className = 'status unknown';
  $('fan').textContent = 'FAN ?';         $('fan').className = 'status unknown';
  for (const id of ['source', 'fog', 'timers', 'cooling', 'dmx', 'sacn', 'artnet', 'ap', 'ip', 'fw']) $(id).textContent = '-';
}

function markLost() {
  if (lost) return;
  lost = true;
  $('conn').textContent = 'connection lost, retrying...';
  setUnknown();
}

function rssiLabel(r) {
  return r >= -50 ? 'excellent' : r >= -60 ? 'very good' : r >= -70 ? 'good' : r >= -80 ? 'weak' : 'poor';
}

function render(d) {
  const manual = (d.source === 'none');   // update and manual control only when no console controls the machine
  $('upd').hidden = !manual;
  $('manual').hidden = !manual;
  $('ready').textContent = d.ready ? 'READY' : 'NOT READY';
  $('ready').className = 'status ' + (d.ready ? 'ready_on' : 'ready_off');
  $('heating').textContent = d.heating ? 'HEATING' : 'NOT HEATING';
  $('heating').className = 'status ' + (d.heating ? 'heating_on' : 'heating_off');
  $('fan').textContent = d.fan ? 'FAN ON' : 'FAN OFF';
  $('fan').className = 'status ' + (d.fan ? 'fan_on' : 'fan_off');
  $('source').textContent = d.source;
  $('fog').textContent = d.fog + ' %';
  $('timers').textContent = d.tOn + ' s / ' + d.tOff + ' s';
  $('cooling').textContent = d.cooling ? 'Yes' : 'No';
  $('dmx').textContent = d.dmxAddress;
  $('sacn').textContent = d.sacnUniverse;
  $('artnet').textContent = d.artnetUniverse;
  let ap = d.ap ? 'ON (1.2.3.4)' : 'OFF';
  if (d.ap) {
    if (d.apClients > 0) {
      ap += ' | ' + d.apRssi + ' dBm (' + rssiLabel(d.apRssi) + ')';
      if (d.apClients > 1) ap += ', weakest of ' + d.apClients + ' clients';
    } else {
      ap += ' | no Wi-Fi client';
    }
  }
  $('ap').textContent = ap;
  $('ip').textContent = d.ip + (d.link ? '' : ' (no link)');
  $('fw').textContent = d.fw;

  if (manual && Date.now() > editHold) {
    $('mFog').value = d.fog;
    $('mFogV').textContent = d.fog + ' %';
    $('mOn').value = d.tOn;
    $('mOff').value = d.tOff;
    $('mFan').checked = d.fan;
  }
  const canFog = manual && d.ready;
  psht.disabled = !canFog;
  psht.textContent = d.psht ? 'FOGGING...' : (d.ready ? 'Hold to fog' : 'Not ready');
  if (!canFog) fogStop(false);
}

// ---- manual settings (fog, timers, fan relay)
function holdEdit() { editHold = Date.now() + 2500; }
function clamp(v, max) { v = parseInt(v, 10); return isNaN(v) ? 0 : Math.min(max, Math.max(0, v)); }

function sendManual() {
  const q = new URLSearchParams({
    fog: clamp($('mFog').value, 100),
    ton: clamp($('mOn').value, 255),
    toff: clamp($('mOff').value, 255),
    fan: $('mFan').checked ? 1 : 0
  });
  fetch('/api/manual?' + q, { method: 'POST' }).catch(() => {});
}
$('mFog').addEventListener('input', () => { holdEdit(); $('mFogV').textContent = $('mFog').value + ' %'; });
$('mOn').addEventListener('input', holdEdit);
$('mOff').addEventListener('input', holdEdit);
// Stop : fog, timer ON and timer OFF back to 0 (the fan relay is left as it is)
$('stop').addEventListener('click', () => {
  fogStop();
  $('mFog').value = 0; $('mFogV').textContent = '0 %';
  $('mOn').value = 0;
  $('mOff').value = 0;
  holdEdit();
  fetch('/api/manual?fog=0&ton=0&toff=0', { method: 'POST' }).catch(() => {});
});
for (const id of ['mFog', 'mOn', 'mOff', 'mFan']) {
  $(id).addEventListener('change', () => { holdEdit(); sendManual(); });
}

// ---- hold to fog : dead-man switch. The ESP32 only fogs while heartbeats keep arriving
function fogBeat() {
  const c = new AbortController();
  const t = setTimeout(() => c.abort(), 800);
  fetch('/api/fog?s=1', { method: 'POST', signal: c.signal })
    .then(r => { clearTimeout(t); if (!r.ok) fogStop(false); })
    .catch(() => { clearTimeout(t); fogStop(false); });   // no answer = link considered lost
}

function fogStart() {
  if (fogTimer || psht.disabled) return;
  fogBeat();
  fogTimer = setInterval(fogBeat, 250);
  psht.classList.add('on');
}

function fogStop(notify = true) {
  if (!fogTimer) return;
  clearInterval(fogTimer);
  fogTimer = null;
  psht.classList.remove('on');
  if (notify) fetch('/api/fog?s=0', { method: 'POST' }).catch(() => {});
}

psht.addEventListener('pointerdown', e => {
  e.preventDefault();
  try { psht.setPointerCapture(e.pointerId); } catch (_) {}
  fogStart();
});
['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => psht.addEventListener(ev, () => fogStop()));
psht.addEventListener('contextmenu', e => e.preventDefault());
window.addEventListener('blur', () => fogStop());
document.addEventListener('visibilitychange', () => { if (document.hidden) fogStop(); });
window.addEventListener('pagehide', () => { if (fogTimer) navigator.sendBeacon('/api/fog?s=0'); });

// ---- live status
function connect() {
  if (src) src.close();
  src = new EventSource('/events');
  src.onerror = markLost;
  src.onmessage = e => {
    lastMsg = Date.now();
    lost = false;
    $('conn').textContent = 'live';
    render(JSON.parse(e.data));
  };
}

// Unplugging the cable does not raise any error on a TCP connection, so we watch
// the heartbeat: no message for TIMEOUT_MS => link considered lost, force a new connection.
setInterval(() => {
  const now = Date.now();
  if (now - lastMsg > TIMEOUT_MS) {
    markLost();
    if (now - lastRetry > TIMEOUT_MS) { lastRetry = now; connect(); }
  }
}, 1000);

connect();
</script>
</body>
</html>
)rawliteral";

const char update_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Firmware update</title>
<style>
[hidden]{display:none!important}
body{font-family:Arial,sans-serif;text-align:center;margin:30px auto;max-width:480px;padding:0 12px}
.small{font-size:.9em;color:gray}
.warn{font-size:.85em;color:#a60;margin:18px 0;text-align:left}
.lock{color:#a60;margin:12px 0}
.err{color:#c00;margin:10px 0;min-height:1.2em}
button,.btn{font-size:1em;padding:8px 18px;margin:6px;border:1px solid #888;border-radius:6px;background:#eee;cursor:pointer;text-decoration:none;color:#000;display:inline-block}
button:disabled{opacity:.4;cursor:default}
.primary{background:#2a7;color:#fff;border-color:#2a7}
.bar{height:16px;background:#ddd;border-radius:8px;overflow:hidden;margin:16px 0 4px}
#fill{height:100%;width:0;background:#2a7;transition:width .2s}
#msg{margin:16px 0}
</style>
</head>
<body>
<h1>Firmware update</h1>
<div id="cur" class="small">Current firmware: ...</div>

<div id="pick">
<p><input type="file" id="file" accept=".bin"></p>
<div id="info" class="small"></div>
<div id="err" class="err"></div>
<div id="lock" class="lock" hidden></div>
<button id="start" class="primary" hidden disabled>Start update</button>
<a class="btn" href="/">Back</a>
<div class="warn">Select the application binary (.bin) exported with "Sketch &gt; Export Compiled Binary", not the merged, bootloader or partitions files. The fog output and the FAN relay are switched off during the update, and the controller restarts when it is finished. The update is only possible when no console controls the machine.</div>
</div>

<div id="run" hidden>
<div id="msg">Update in progress, see fog machine screen for status. Once the update is done, reload the page.</div>
<div class="bar"><div id="fill"></div></div>
<div id="pct" class="small">0 %</div>
<button id="reload" onclick="location.href='/'">Reload</button>
<button id="retry" onclick="location.href='/update'" hidden>Try again</button>
</div>

<script>
const $ = id => document.getElementById(id);
const f = $('file'), start = $('start');
let ctrlOk = false;   // no console controlling the machine
let fileOk = false;   // a valid .bin is selected
let running = false;  // upload started

function refreshUI() {
  start.hidden = !ctrlOk;
  start.disabled = !fileOk;
  $('lock').hidden = ctrlOk;
}

// Control source is checked every 2 s: the "Start update" button only exists while it is "none"
function pollStatus() {
  if (running) return;
  fetch('/api/status', { cache: 'no-store' }).then(r => r.json()).then(d => {
    $('cur').textContent = 'Current firmware: v' + d.fw;
    ctrlOk = (d.source === 'none');
    $('lock').textContent = ctrlOk ? '' :
      'Update not available: the machine is currently controlled by ' + d.source + '. Stop the console output, this page unlocks by itself.';
    refreshUI();
  }).catch(() => {
    ctrlOk = false;
    $('lock').textContent = 'Update not available: cannot read the machine status.';
    refreshUI();
  });
}
pollStatus();
setInterval(pollStatus, 2000);

f.onchange = () => {
  fileOk = false; $('err').textContent = ''; $('info').textContent = '';
  const file = f.files[0];
  if (file) {
    const n = file.name.toLowerCase();
    if (!n.endsWith('.bin')) {
      $('err').textContent = 'Please select a .bin file.';
    } else if (/merged|bootloader|partitions/.test(n)) {
      $('err').textContent = 'This is not the application image: use the .bin without "merged", "bootloader" or "partitions" in its name.';
    } else {
      $('info').textContent = file.name + ' (' + Math.round(file.size / 1024) + ' KB)';
      fileOk = true;
    }
  }
  refreshUI();
};

function setMsg(t) { $('msg').textContent = t; }

function fail(t) {
  $('fill').style.background = '#c00';
  setMsg('Update failed: ' + t);
  $('retry').hidden = false;
}

function done() {
  $('fill').style.width = '100%';
  $('pct').textContent = '';
  setMsg('Update complete, the controller has restarted. Click Reload.');
  $('reload').className = 'primary';
}

// After the upload the controller reboots: wait until it answers again
function waitForDevice() {
  const poll = () => {
    const c = new AbortController();
    const t = setTimeout(() => c.abort(), 2000);
    fetch('/api/status', { cache: 'no-store', signal: c.signal })
      .then(r => { clearTimeout(t); if (r.ok) done(); else setTimeout(poll, 1000); })
      .catch(() => { clearTimeout(t); setTimeout(poll, 1000); });
  };
  setTimeout(poll, 4000);
}

function upload(file) {
  running = true;
  $('pick').hidden = true;
  $('run').hidden = false;

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update?size=' + file.size);
  xhr.upload.onprogress = e => {
    if (e.lengthComputable) {
      const p = Math.round(e.loaded * 100 / e.total);
      $('fill').style.width = p + '%';
      $('pct').textContent = p + ' %';
    }
  };
  xhr.onload = () => {
    if (xhr.status === 200) {
      setMsg('Firmware sent, the controller is restarting...');
      $('pct').textContent = 'restarting...';
      waitForDevice();
    } else {
      fail(xhr.responseText || ('HTTP ' + xhr.status));
    }
  };
  xhr.onerror = () => fail('connection error during the upload.');
  const fd = new FormData();
  fd.append('firmware', file, file.name);
  xhr.send(fd);
}

start.onclick = () => {
  const file = f.files[0];
  if (!file) return;
  // Last check right before sending (the ESP32 checks again on its side)
  fetch('/api/status', { cache: 'no-store' }).then(r => r.json()).then(d => {
    if (d.source !== 'none') { pollStatus(); return; }
    if (!confirm('Start the update? The fog output will stay off until the controller has restarted.')) return;
    upload(file);
  }).catch(() => { $('err').textContent = 'Cannot read the machine status.'; });
};
</script>
</body>
</html>
)rawliteral";

//=============================================================================
// Settings load / save
//=============================================================================
static void loadIp(const char *key, uint8_t *dst, const uint8_t *def) {
  if (preferences.getBytesLength(key) == 4) preferences.getBytes(key, dst, 4);
  else memcpy(dst, def, 4);
}

void loadSettings() {
  preferences.begin("params", false);

  uint32_t v = preferences.getUInt("dmxStartAddr", 1);
  DMXstartAddress = (v >= 1 && v <= 509) ? v : 1;

  v = preferences.getUInt("univ", UNIVERSE);
  sacnUniverse = (v >= 1 && v <= 63999) ? v : UNIVERSE;

  v = preferences.getUInt("artUniv", ARTNET_DEFAULT_UNIVERSE);
  artnetUniverse = (v <= 32767) ? v : ARTNET_DEFAULT_UNIVERSE;

  // New key names on purpose: the previous firmware stored the raw IPAddress object
  loadIp("ipAddr", cfgIp, DEF_IP);
  loadIp("netmask", cfgMask, DEF_MASK);
  loadIp("gateway", cfgGw, DEF_GW);

  wifiApEnabled = preferences.getBool("wifiAp", false);

  preferences.end();
}

void saveSettings() {
  preferences.begin("params", false);
  preferences.putUInt("dmxStartAddr", DMXstartAddress);
  preferences.putUInt("univ", sacnUniverse);
  preferences.putUInt("artUniv", artnetUniverse);
  preferences.putBytes("ipAddr", cfgIp, 4);
  preferences.putBytes("netmask", cfgMask, 4);
  preferences.putBytes("gateway", cfgGw, 4);
  preferences.putBool("wifiAp", wifiApEnabled);
  preferences.end();
}

bool networkChangedSinceBoot() {
  return memcmp(cfgIp, bootIp, 4) || memcmp(cfgMask, bootMask, 4) ||
         memcmp(cfgGw, bootGw, 4) || sacnUniverse != bootSacnUniverse;
}

//=============================================================================
// Status JSON
//=============================================================================
String getStatusJSON() {
  JsonDocument doc;

  doc["fw"] = String(FW_MAJOR) + "." + String(FW_MINOR);
  doc["source"] = sourceName(activeSource);
  doc["dmxWired"] = (bool)dmxIsConnected;
  doc["ready"] = (bool)foggerReady;
  doc["heating"] = (bool)foggerHeating;
  doc["fan"] = (bool)fanRelay;
  doc["fog"] = fogIntensity;
  doc["tOn"] = tOn;
  doc["tOff"] = tOff;
  doc["cooling"] = (bool)isCoolingDown;
  doc["dmxAddress"] = DMXstartAddress;
  doc["sacnUniverse"] = sacnUniverse;
  doc["artnetUniverse"] = artnetUniverse;
  doc["psht"] = (bool)forceFullFog;     // full fog ("Pshiiit") currently active
  doc["ap"] = wifiApActive;
  if (wifiApActive) {
    // Signal strength of the Wi-Fi clients, as received by the machine (weakest client)
    wifi_sta_list_t sta;
    int n = 0, worst = 0;
    if (esp_wifi_ap_get_sta_list(&sta) == ESP_OK) {
      n = sta.num;
      for (int i = 0; i < n; i++) {
        if (i == 0 || sta.sta[i].rssi < worst) worst = sta.sta[i].rssi;
      }
    }
    doc["apClients"] = n;
    if (n > 0) doc["apRssi"] = worst;
  }

  char ipb[16];
  snprintf(ipb, sizeof(ipb), "%u.%u.%u.%u", bootIp[0], bootIp[1], bootIp[2], bootIp[3]);
  doc["ip"] = String(ipb);
  doc["link"] = networkStarted && ETH.linkUp();

  String json;
  serializeJson(doc, json);
  return json;
}

//=============================================================================
// Network callbacks (called from the AsyncUDP task: keep them short)
//=============================================================================
void onSacnPacket(e131_packet_t *p, void *) {
  if (ntohs(p->universe) != sacnUniverse) return;
  uint16_t n = ntohs(p->property_value_count);     // includes start code, validated by the lib
  if (n > DMX_PACKET_SIZE) n = DMX_PACKET_SIZE;
  memcpy(sacnData, p->property_values, n);
  lastSacnMs = millis();
}

void onArtNetPacket(uint16_t portAddress, const uint8_t *data, uint16_t len) {
  if (portAddress != artnetUniverse) return;
  artnetData[0] = 0;
  memcpy(artnetData + 1, data, len);               // len <= 512, validated by the class
  lastArtnetMs = millis();
}

//=============================================================================
// Firmware update (OTA over Ethernet)
//=============================================================================
// Called from the async_tcp task for each chunk of the uploaded file.
void handleOtaUpload(AsyncWebServerRequest *request, const String &filename, size_t index,
                     uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    if (otaRejectedReq == request) otaRejectedReq = nullptr;   // stale pointer from an interrupted request

    // Refuse the update while a console controls the machine (or if another update is running)
    if (otaInProgress || activeSource != SRC_NONE) {
      otaRejectedReq = request;
      otaRejectMsg = otaInProgress ? "An update is already in progress"
                                   : "Update refused: the machine is controlled by a console";
      if (!otaInProgress) showMsg("UPD DENY", 2000);
      Serial.printf("OTA refused: %s\n", otaRejectMsg);
      return;
    }

    // Safe state while the flash is being written
    otaInProgress = true;
    otaFailed = false;
    otaErrorMsg = "";
    otaPercent = 0;
    menuState = MENU_HOME;
    ledcWrite(PWM_OUT, 0);
    digitalWrite(FAN_RELAY_PIN, LOW);

    size_t total = UPDATE_SIZE_UNKNOWN;
    if (request->hasParam("size")) {                 // sent by the update page: exact size
      const long sz = request->getParam("size")->value().toInt();
      if (sz > 0) total = sz;
    }
    otaTotal = (total == UPDATE_SIZE_UNKNOWN) ? 0 : total;
    Serial.printf("OTA start: %s (%u bytes)\n", filename.c_str(), (unsigned)otaTotal);

    if (!Update.begin(total)) {
      otaFailed = true;
      otaErrorMsg = Update.errorString();
      Update.printError(Serial);
    }
  }

  if (otaRejectedReq == request) return;             // refused request: ignore the rest of the body

  otaLastChunkMs = millis();
  if (otaFailed) return;

  if (len > 0 && Update.write(data, len) != len) {
    otaFailed = true;
    otaErrorMsg = Update.errorString();
    Update.printError(Serial);
    Update.abort();
    return;
  }

  if (otaTotal) {
    uint32_t pct = ((uint64_t)(index + len) * 100) / otaTotal;
    otaPercent = (pct > 99) ? 99 : pct;              // 100 % only once the image is validated
  }

  if (final) {
    if (Update.end(true)) {
      otaPercent = 100;
      Serial.println("OTA success");
    } else {
      otaFailed = true;
      otaErrorMsg = Update.errorString();
      Update.printError(Serial);
    }
  }
}

//=============================================================================
// Wi-Fi access point (optional, ON/OFF in the OLED menu) : http://1.2.3.4
//=============================================================================
void applyWifiAp() {
  if (wifiApEnabled && !wifiApActive) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(1, 2, 3, 4), IPAddress(1, 2, 3, 4), IPAddress(255, 255, 255, 0));
    if (WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS)) {
      wifiApActive = true;
      Serial.print("Wi-Fi AP started, SSID: ");
      Serial.print(AP_SSID);
      Serial.print("  IP: ");
      Serial.println(WiFi.softAPIP());
    } else {
      WiFi.mode(WIFI_OFF);
      Serial.println("*** Wi-Fi AP start failed (password must have 8 characters minimum) ***");
    }
  }
  else if (!wifiApEnabled && wifiApActive) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiApActive = false;
    Serial.println("Wi-Fi AP stopped");
  }
}

//=============================================================================
// Network start (W5500 through ETH.h + sACN + Art-Net + web server)
//=============================================================================
void startWebServer() {
  events.onConnect([](AsyncEventSourceClient *client) {
    client->send(getStatusJSON().c_str(), NULL, millis(), 1000);
  });
  server.addHandler(&events);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getStatusJSON());
  });

  // Manual settings (only while no console controls the machine)
  server.on("/api/manual", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (activeSource != SRC_NONE || otaInProgress) {
      request->send(409, "text/plain", "The machine is not in manual mode");
      return;
    }
    if (auto *p = request->getParam("fog"))  fogIntensity = constrain(p->value().toInt(), 0, 100);
    if (auto *p = request->getParam("ton"))  tOn  = constrain(p->value().toInt(), 0, 255);
    if (auto *p = request->getParam("toff")) tOff = constrain(p->value().toInt(), 0, 255);
    if (auto *p = request->getParam("fan"))  fanRelay = (p->value().toInt() != 0);
    request->send(200, "application/json", getStatusJSON());
  });

  // "Hold to fog" from the page. Dead-man switch : full fog only lasts WEB_FOG_TIMEOUT_MS after the
  // last heartbeat (s=1, sent every 250 ms while the button is held). A release (s=0), a lost
  // connection or a closed page therefore stops the fog on its own, and one press can never exceed
  // WEB_FOG_MAX_MS. The same conditions as the local PLUS button apply (manual mode, machine ready).
  server.on("/api/fog", HTTP_POST, [](AsyncWebServerRequest *request) {
    const uint32_t now = millis();
    const auto *p = request->getParam("s");
    const bool start = (p && p->value().toInt() == 1);

    if (!start) {                                   // release: stop at once
      webFogUntil = now;
      webFogStopMs = now;
      request->send(200, "text/plain", "OK");
      return;
    }
    if (activeSource != SRC_NONE || otaInProgress) {
      webFogUntil = now;
      request->send(409, "text/plain", "The machine is not in manual mode");
      return;
    }
    if (webFogStopMs && (now - webFogStopMs) < 300) {   // heartbeat overtaken by the release: ignore
      request->send(200, "text/plain", "OK");
      return;
    }
    if ((int32_t)(webFogUntil - now) <= 0) webFogStart = now;   // new press
    webFogUntil = now + WEB_FOG_TIMEOUT_MS;
    request->send(200, "text/plain", "OK");
  });

  // Firmware update page
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", update_html);
  });

  // Firmware upload (no authentication: only use on a closed network)
  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (otaRejectedReq == request) {
        otaRejectedReq = nullptr;
        AsyncWebServerResponse *resp = request->beginResponse(409, "text/plain", String(otaRejectMsg));
        resp->addHeader("Connection", "close");
        request->send(resp);
        return;
      }
      if (!otaInProgress) {
        request->send(400, "text/plain", "No firmware file received");
        return;
      }
      if (otaFailed) {
        if (Update.isRunning()) Update.abort();
        otaInProgress = false;                       // outputs are released again
        showMsg("UPD FAIL", 3000);
        AsyncWebServerResponse *resp = request->beginResponse(500, "text/plain", String(otaErrorMsg));
        resp->addHeader("Connection", "close");
        request->send(resp);
        return;
      }
      AsyncWebServerResponse *resp = request->beginResponse(200, "text/plain", "OK");
      resp->addHeader("Connection", "close");
      request->send(resp);
      rebootAt = millis() + 1500;
      rebootRequested = true;
    },
    handleOtaUpload);

  server.begin();
}

void startNetwork() {
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);

  if (!ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI)) {
    Serial.println("HW ERROR : Ethernet interface not found.");
    return;
  }

  // Fixed address (call config() right after begin() to stop DHCP)
  IPAddress ip(cfgIp[0], cfgIp[1], cfgIp[2], cfgIp[3]);
  IPAddress mask(cfgMask[0], cfgMask[1], cfgMask[2], cfgMask[3]);
  IPAddress gw(cfgGw[0], cfgGw[1], cfgGw[2], cfgGw[3]);
  ETH.config(ip, gw, mask, gw);   // we assume the DNS server is the gateway
  networkStarted = true;

  Serial.print("Ethernet IP: ");
  Serial.println(ETH.localIP());

  // sACN (multicast group of the universe + unicast on UDP 5568)
  e131.registerCallback(nullptr, onSacnPacket);
  if (e131.begin(E131_MULTICAST, sacnUniverse, UNIVERSE_COUNT)) Serial.println(F("sACN: listening"));
  else Serial.println(F("*** sACN begin failed ***"));

  // Art-Net (UDP 6454, broadcast + unicast)
  if (artnet.begin(onArtNetPacket, "Magnum850", "Martin Magnum 850 Fog Machine")) Serial.println(F("Art-Net: listening"));
  else Serial.println(F("*** Art-Net begin failed ***"));

  startWebServer();
  applyWifiAp();                  // Wi-Fi access point, if enabled in the menu
}

//=============================================================================
// Menu helpers
//=============================================================================
uint8_t *itemIpPtr(uint8_t it) { return it == IT_IP ? cfgIp : (it == IT_MASK ? cfgMask : cfgGw); }
bool itemIsIp(uint8_t it) { return it >= IT_IP && it <= IT_GW; }
// Fog / timers / fan are only editable by hand when no remote source drives the machine
bool itemLocked(uint8_t it) { return it <= IT_FAN && activeSource != SRC_NONE; }

void itemValueStr(uint8_t it, char *buf, size_t n) {
  if (itemLocked(it)) { snprintf(buf, n, "remote"); return; }
  switch (it) {
    case IT_FOG:    snprintf(buf, n, "%u%%", (unsigned)fogIntensity); break;
    case IT_TON:    snprintf(buf, n, "%us", (unsigned)tOn); break;
    case IT_TOFF:   snprintf(buf, n, "%us", (unsigned)tOff); break;
    case IT_DMX:    snprintf(buf, n, "%u", (unsigned)DMXstartAddress); break;
    case IT_SACN:   snprintf(buf, n, "%u", (unsigned)sacnUniverse); break;
    case IT_ARTNET: snprintf(buf, n, "%u", (unsigned)artnetUniverse); break;
    case IT_FAN:    snprintf(buf, n, "%s", fanRelay ? "ON" : "OFF"); break;
    case IT_WIFI:   snprintf(buf, n, "%s", wifiApEnabled ? "ON" : "OFF"); break;
    case IT_IP:
    case IT_MASK:
    case IT_GW: {
      const uint8_t *p = itemIpPtr(it);
      snprintf(buf, n, "%u.%u.%u.%u", p[0], p[1], p[2], p[3]);
      break;
    }
    default: buf[0] = 0;
  }
}

void showMsg(const char *text, uint32_t durationMs) {
  msgText = text;
  msgUntil = millis() + durationMs;
}

void editValue(bool plus, bool minus) {
  switch (menuIndex) {
    case IT_FOG:    if (plus && fogIntensity < 100) fogIntensity++; if (minus && fogIntensity > 0) fogIntensity--; break;
    case IT_TON:    if (plus && tOn < 255) tOn++;   if (minus && tOn > 0) tOn--;   break;
    case IT_TOFF:   if (plus && tOff < 255) tOff++; if (minus && tOff > 0) tOff--; break;
    case IT_DMX:    if (plus && DMXstartAddress < 509) DMXstartAddress++; if (minus && DMXstartAddress > 1) DMXstartAddress--; break;
    case IT_SACN:   if (plus && sacnUniverse < 63999) sacnUniverse++;     if (minus && sacnUniverse > 1) sacnUniverse--; break;
    case IT_ARTNET: if (plus && artnetUniverse < 32767) artnetUniverse++; if (minus && artnetUniverse > 0) artnetUniverse--; break;
    case IT_FAN:    if (plus) fanRelay = true;       if (minus) fanRelay = false;       break;   // + = ON, - = OFF
    case IT_WIFI:   if (plus) wifiApEnabled = true;  if (minus) wifiApEnabled = false;  break;
    case IT_IP:
    case IT_MASK:
    case IT_GW: {                    // IP / mask / gateway : octets wrap around 255 <-> 0
      uint8_t *p = itemIpPtr(menuIndex);
      if (plus) p[editIndex]++;
      if (minus) p[editIndex]--;
      break;
    }
  }
}

void exitMenu(bool save) {
  menuState = MENU_HOME;
  if (save) {
    saveSettings();
    if (wifiApEnabled != wifiApActive) wifiApApplyRequested = true;   // applied by loop()
    if (networkChangedSinceBoot()) {
      showMsg("REBOOT", 1500);
      rebootAt = millis() + 1500;
      rebootRequested = true;
    } else {
      showMsg("SAVED", 1000);
    }
  } else {
    loadSettings();                  // timeout: cancel the changes
    showMsg("CANCEL", 800);
  }
}

void handleMenu() {
  const bool plus = btnPlusShortPress, minus = btnMinusShortPress;
  const bool ent = btnEnterShortPress, hold = btnEnterLongPress;

  switch (menuState) {
    case MENU_HOME:
      if (hold) { menuState = MENU_MAIN; menuIndex = 0; }
      break;

    case MENU_MAIN:
      if (hold) { exitMenu(true); break; }
      if (plus && menuIndex < IT_COUNT - 1) menuIndex++;
      if (minus && menuIndex > 0) menuIndex--;
      if (ent && !itemLocked(menuIndex)) { menuState = MENU_EDIT; editIndex = 0; }
      break;

    case MENU_EDIT:
      if (hold) { exitMenu(true); break; }
      if (itemLocked(menuIndex)) { menuState = MENU_MAIN; break; }
      editValue(plus, minus);
      if (ent) {
        if (itemIsIp(menuIndex) && editIndex < 3) editIndex++;
        else menuState = MENU_MAIN;
      }
      break;
  }
}

bool repeatPress(bool pressed, RepeatBtn &b, unsigned long now) {
  if (!pressed) { b.held = false; return false; }
  if (!b.held) { b.held = true; b.t0 = b.tLast = now; return true; }   // first immediate step

  const unsigned long heldTime = now - b.t0;
  if (heldTime <= REPEAT_START_DELAY) return false;

  const unsigned long accel = heldTime - REPEAT_START_DELAY;
  unsigned long interval;
  if (accel >= REPEAT_ACCEL_TIME) interval = REPEAT_FAST_INTERVAL;
  else interval = REPEAT_INITIAL_INTERVAL - ((REPEAT_INITIAL_INTERVAL - REPEAT_FAST_INTERVAL) * accel) / REPEAT_ACCEL_TIME;

  if (now - b.tLast >= interval) { b.tLast = now; return true; }
  return false;
}

//=============================================================================
// Source selection : wired DMX > sACN > Art-Net
//=============================================================================
uint8_t pickSource() {
  const uint32_t now = millis();
  if (dmxIsConnected) return SRC_DMX;
  if (lastSacnMs && (now - lastSacnMs) < NET_TIMEOUT_SACN_MS) return SRC_SACN;
  if (lastArtnetMs && (now - lastArtnetMs) < NET_TIMEOUT_ARTNET_MS) return SRC_ARTNET;
  return SRC_NONE;
}

//=============================================================================
// Tasks
//=============================================================================
void TaskDMXwired(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskInput(void *pvParameters);
void TaskPWM(void *pvParameters);

void setup() {
  Serial.begin(115200);

  pinMode(SW_MINUS, INPUT_PULLUP);
  pinMode(SW_ENTER, INPUT); //HW pullup because it uses GPIO35 (no internal pullup or pulldown)
  pinMode(SW_PLUS, INPUT_PULLUP);
  pinMode(FOGGER_HEATING, INPUT_PULLUP);
  pinMode(FOGGER_READY, INPUT_PULLUP);
  pinMode(FAN_RELAY_PIN, OUTPUT);

  ledcAttach(PWM_OUT, PWM_FREQ, 8);
  ledcWrite(PWM_OUT, 0);

  // Splash screen
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Martin Magnum");
  display.println("DMX controller");
  display.println("");
  display.print("v.");
  display.print(FW_MAJOR);
  display.print(".");
  display.print(FW_MINOR);
  display.display();

  //Retrieve parameters from memory
  loadSettings();
  memcpy(bootIp, cfgIp, 4);
  memcpy(bootMask, cfgMask, 4);
  memcpy(bootGw, cfgGw, 4);
  bootSacnUniverse = sacnUniverse;

  //DMX initialization
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };
  int personality_count = 1;
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);

  startNetwork();

  delay(2000);
  display.clearDisplay();

  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(TaskPWM,       "TaskPWM",       2048, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(TaskDMXwired,  "TaskDMXwired",  2048, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(TaskDisplay,   "TaskDisplay",   4096, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(TaskInput,     "TaskInput",     4096, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
}

// loop() : web status push + deferred reboot (the web server and UDP are event driven)
void loop() {
  static uint32_t lastCheck = 0, lastPush = 0;
  static String lastJson;
  const uint32_t now = millis();

  if (rebootRequested && (int32_t)(now - rebootAt) >= 0) {
    ESP.restart();
  }

  if (wifiApApplyRequested && networkStarted) {
    wifiApApplyRequested = false;
    applyWifiAp();
  }

  // Upload interrupted (browser closed, cable unplugged...): give up and release the outputs
  if (otaInProgress && !rebootRequested && (now - otaLastChunkMs) > 10000) {
    if (Update.isRunning()) Update.abort();
    otaInProgress = false;
    showMsg("UPD ABORT", 3000);
    Serial.println("OTA aborted: upload interrupted");
  }

  if (now - lastCheck >= 250) {
    lastCheck = now;

    artnet.setPortAddress(artnetUniverse);
    artnet.dataActive = (activeSource == SRC_ARTNET);

    if (networkStarted && events.count() > 0) {
      String json = getStatusJSON();
      if (json != lastJson || (now - lastPush) > 2000) {
        events.send(json.c_str());
        lastJson = json;
        lastPush = now;
      }
    }
  }
  delay(20);
}

void TaskDMXwired(void *pvParameters) { // Take care of wired DMX IN packets
  (void) pvParameters;

  while (1) {
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) { //DMX data received
      if (!packet.err) {
        if (!dmxIsConnected) {
          Serial.print("DMX is connected! Listening to channels starting at ");
          Serial.println(DMXstartAddress);
          dmxIsConnected = true;
        }
        dmx_read(dmxPort, DMXdata, packet.size);
      } else {
        Serial.println("A DMX error occurred.");
      }
    } else if (dmxIsConnected) {
      Serial.println("DMX was disconnected.");
      dmxIsConnected = false;
    }
    yield();
  }
}

void TaskPWM(void *pvParameters) { // Compute the fog output from the active control source
  (void) pvParameters;
  uint8_t prevSource = SRC_NONE;
  uint8_t lastPWM = 255;

  while (1) {
    const uint8_t src = pickSource();
    activeSource = src;

    if (src != SRC_NONE) {
      const uint8_t *d = (src == SRC_DMX) ? DMXdata : ((src == SRC_SACN) ? sacnData : artnetData);
      const uint16_t a = DMXstartAddress;   // 1..509, so a+3 <= 512

      fogIntensity = map(d[a], 0, 255, 0, 100); //converts fog intensity in percent
      tOn = d[a + 1];
      tOff = d[a + 2];
      //Special channels settings for triggering Cool down mode
      isCoolingDown = (d[a] == 0 && d[a + 1] == 0 && d[a + 2] == 255);
      fanRelay = (d[a + 3] > 127);
    }
    else if (prevSource != SRC_NONE) {
      // Control signal lost : go back to a safe idle state
      fogIntensity = 0;
      tOn = 0;
      tOff = 0;
      isCoolingDown = false;
      fanRelay = false;
    }
    prevSource = src;

    uint8_t PWMvalue = map(fogIntensity, 0, 100, VOUT_START, VOUT_END);

    if (tOn == 0 || tOff == 0) { //Force to ON (continuous)
      PWMactivated = true;
    }
    else if (!PWMactivated && (millis() - fogOffTimer) > ((unsigned long)tOff) * 1000) { //Timer OFF is over, switch to ON
      PWMactivated = true;
      fogOnTimer = millis();
    }
    else if (PWMactivated && (millis() - fogOnTimer) > ((unsigned long)tOn) * 1000) { //Timer ON is over, switch to OFF
      PWMactivated = false;
      fogOffTimer = millis();
    }

    if (isCoolingDown) {
      PWMvalue = 0;
    }
    else if (PWMactivated || forceFullFog) {
      if (forceFullFog) {
        PWMvalue = VOUT_END;
      }
      else if (fogIntensity == 0) {
        PWMvalue = VOUT_IDLE;
      }
    }
    else {
      PWMvalue = VOUT_IDLE;
    }

    if (otaInProgress) PWMvalue = 0;                 // firmware update : outputs forced to 0
    ledcWrite(PWM_OUT, PWMvalue);
    digitalWrite(FAN_RELAY_PIN, otaInProgress ? LOW : fanRelay);

#ifdef DEBUG
    if (PWMvalue != lastPWM) {
      Serial.print("PWM Value: ");
      Serial.print(PWMvalue);
      Serial.print("  source: ");
      Serial.println(sourceName(src));
      lastPWM = PWMvalue;
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//=============================================================================
// Display
//=============================================================================
void drawMsg() {
  const char *t = msgText;
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  const int w = strlen(t) * 12;
  display.setCursor((SCREEN_WIDTH - w) / 2, 9);
  display.print(t);
}

void drawHome() {
  const bool remote = (activeSource != SRC_NONE);
  display.setTextSize(1);

  // Bottom line : source, or alternating "NO SIGNAL" / IP address
  display.setCursor(0, 24);
  if (remote) {
    display.print(sourceName(activeSource));
    display.print(F(" CONNECTED"));
  }
  else if (flag1s) {
    display.print(F("NO SIGNAL"));
  }
  else if (networkStarted && ETH.linkUp()) {
    char b[16];
    snprintf(b, sizeof(b), "%u.%u.%u.%u", bootIp[0], bootIp[1], bootIp[2], bootIp[3]);
    display.print(b);
  }
  else {
    display.print(F("NO LINK"));
  }

  // FAN relay indicator (bottom right)
  if (fanRelay) {
    display.setTextSize(1);
    display.setCursor(104, 24);
    display.print(F("FAN"));
  }

  // Left side of display
  if (isCoolingDown) {
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.println("COOLING");
  }
  else if (forceFullFog) {
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.println("PSHHH!");
  }
  else if (!remote && fogIntensity == 0) {
    display.setTextSize(2);
    display.setCursor(12, 5);
    display.println("IDLE");
  }
  else {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("FOG : ");
    display.print(fogIntensity);
    display.println("%");
    display.print("TON : ");
    display.print(tOn);
    display.println("s");
    display.print("TOFF: ");
    display.print(tOff);
    display.println("s");
  }

  // Right side of display
  display.setCursor(102, 0);
  display.setTextSize(1);
  display.print("CH");
  display.setTextSize(2);
  if (DMXstartAddress < 10) { display.setCursor(102, 8); }
  else if (DMXstartAddress < 100) { display.setCursor(96, 8); }
  else { display.setCursor(90, 8); }
  display.print(DMXstartAddress);
}

void drawMenuList() {
  display.setTextSize(1);
  const int first = constrain((int)menuIndex - 1, 0, IT_COUNT - 4);
  for (int row = 0; row < 4; row++) {
    const int it = first + row;
    char val[20];
    char line[28];
    itemValueStr(it, val, sizeof(val));
    snprintf(line, sizeof(line), "%c%-5s%15s", it == menuIndex ? '>' : ' ', ITEM_LABELS[it], val);
    display.setCursor(0, row * 8);
    display.print(line);
  }
}

void drawEdit() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(ITEM_LABELS[menuIndex]);

  if (itemIsIp(menuIndex)) {
    const uint8_t *p = itemIpPtr(menuIndex);
    for (int i = 0; i < 4; i++) {
      const int x = 19 + i * 24;
      char o[8];
      snprintf(o, sizeof(o), "%3u", p[i]);
      if (i == editIndex) {
        display.fillRect(x - 1, 12, 20, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(x, 13);
      display.print(o);
      display.setTextColor(SSD1306_WHITE);
      if (i < 3) { display.setCursor(x + 18, 13); display.print('.'); }
    }
    display.setCursor(0, 24);
    display.print(F("ENT:next  HOLD:save"));
  }
  else {
    char val[20];
    itemValueStr(menuIndex, val, sizeof(val));
    display.setTextSize(2);
    display.setCursor((SCREEN_WIDTH - (int)strlen(val) * 12) / 2, 8);
    display.print(val);
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(F("ENT:back  HOLD:save"));
  }
}

void drawOta() {
  const uint8_t pct = otaPercent;
  char b[8];
  snprintf(b, sizeof(b), "%u%%", (unsigned)pct);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(pct >= 100 ? F("REBOOTING...") : F("UPDATING..."));
  display.setCursor(SCREEN_WIDTH - (int)strlen(b) * 6, 0);
  display.print(b);

  display.drawRect(0, 11, SCREEN_WIDTH, 12, SSD1306_WHITE);
  display.fillRect(2, 13, ((SCREEN_WIDTH - 4) * pct) / 100, 8, SSD1306_WHITE);

  display.setCursor(0, 25);
  display.print(F("Do not power off"));
}

void TaskDisplay(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    flag1s = ((millis() / 1000) % 2) != 0;   // blinking flag

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (otaInProgress) {
      drawOta();
    }
    else if ((int32_t)(msgUntil - millis()) > 0) {
      drawMsg();
    }
    else {
      switch (menuState) {
        case MENU_MAIN: drawMenuList(); break;
        case MENU_EDIT: drawEdit();     break;
        default:        drawHome();     break;
      }
    }

    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//=============================================================================
// Inputs (buttons + fogger status)
//=============================================================================
void TaskInput(void *pvParameters) { // Take care of digital inputs (user switch panel)
  (void) pvParameters;
  RepeatBtn plusRep, minusRep;

  while (1) {
    const unsigned long now = millis();
    const bool btnMinus = !digitalRead(SW_MINUS);
    const bool btnEnter = !digitalRead(SW_ENTER);
    const bool btnPlus  = !digitalRead(SW_PLUS);

    // ENTER : short press on release, long press once after LONGPRESSTIMER
    if (btnEnter) {
      if (!btnEnterMemo) {                      // rising edge
        btnEnterTimer = now;
        btnEnterLongPressMemo = false;
      }
      else if (!btnEnterLongPressMemo && (now - btnEnterTimer) >= LONGPRESSTIMER) {
        btnEnterLongPress = true;
        btnEnterLongPressMemo = true;
      }
    }
    else if (btnEnterMemo && !btnEnterLongPressMemo) {   // falling edge, no long press
      btnEnterShortPress = true;
    }
    btnEnterMemo = btnEnter;

    // PLUS / MINUS with accelerating auto-repeat
    if (repeatPress(btnPlus, plusRep, now))   btnPlusShortPress = true;
    if (repeatPress(btnMinus, minusRep, now)) btnMinusShortPress = true;

    // Fogger status inputs
    foggerHeating = !digitalRead(FOGGER_HEATING);
    foggerReady = !digitalRead(FOGGER_READY);

    if (otaInProgress) {                          // buttons disabled during a firmware update
      forceFullFog = false;
      btnEnterLongPress = btnEnterShortPress = btnMinusShortPress = btnPlusShortPress = false;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const bool activity = btnPlusShortPress || btnMinusShortPress || btnEnterShortPress || btnEnterLongPress;

    handleMenu();

    // Leave the menu after a while without activity (changes are cancelled)
    if (menuState != MENU_HOME) {
      if (activity) inMenuTimer = now;
      else if ((now - inMenuTimer) > TIMEOUT_MENU) exitMenu(false);
    }
    else {
      inMenuTimer = now;
    }

    // Full fog ("Pshiiit") : hold PLUS on the home screen, or hold the button on the web page.
    // Only when no remote source drives the machine and the fogger is ready.
    // The web request is a dead-man switch: it expires by itself if the heartbeats stop.
    const bool webFog = ((int32_t)(webFogUntil - now) > 0) && ((now - webFogStart) < WEB_FOG_MAX_MS);
    forceFullFog = (activeSource == SRC_NONE && foggerReady) &&
                   ((menuState == MENU_HOME && btnPlus) || webFog);

    btnEnterLongPress = btnEnterShortPress = btnMinusShortPress = btnPlusShortPress = false;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
