#include <Keypad.h>
#include <TM1637Display.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

// -------------------- PINOUT --------------------
#define TM_CLK 18
#define TM_DIO 17

#define BUZZ_PIN 4

#define LED1_PIN 21
#define LED2_PIN 14
#define LED_COUNT 1

#define LID_PIN 16

#define WIRE_OK_PIN 5
#define WIRE_BAD1_PIN 19
#define WIRE_BAD2_PIN 20

// -------------------- WIFI AP --------------------
WebServer server(80);
String apSsid;
String apPass;

// -------------------- KEYPAD --------------------
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


byte rowPins[ROWS] = {6, 7, 15, 8};
byte colPins[COLS] = {9, 10, 11, 12};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// -------------------- DEVICES --------------------
TM1637Display seg(TM_CLK, TM_DIO);
Adafruit_NeoPixel led1(LED_COUNT, LED1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led2(LED_COUNT, LED2_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// -------------------- SETTINGS --------------------
struct Settings {
  char disarmCode[12];
  char adminCode[12];
  uint32_t countdownSec; // max 5999 (99:59)
  uint8_t  maxErrors;
  uint16_t penaltySec;
  uint8_t  mode;         // 1=solo codice, 2=codice->wire stage
  uint32_t wireStageSec; // max 5999
  uint8_t  startOnOpen;  // 1 apertura, 0 chiusura
} cfg;

void loadDefaults() {
  strncpy(cfg.disarmCode, "1234", sizeof(cfg.disarmCode));
  strncpy(cfg.adminCode,  "0000", sizeof(cfg.adminCode));
  cfg.countdownSec = 3 * 60;   // default: 3 minuti
  cfg.maxErrors = 3;
  cfg.penaltySec = 15;
  cfg.mode = 2;          // default: include wire stage (puoi mettere 1)
  cfg.wireStageSec = 2 * 60; // default: 2 minuti
  cfg.startOnOpen = 1;
}

void clampSettings() {
  const uint32_t maxSec = 99 * 3600UL + 59 * 60UL; // 99:59 max
  if (cfg.countdownSec > maxSec) cfg.countdownSec = maxSec;
  if (cfg.wireStageSec > maxSec) cfg.wireStageSec = maxSec;
  if (cfg.mode < 1 || cfg.mode > 2) cfg.mode = 2; // default prefer mode 2
  if (cfg.maxErrors > 9) cfg.maxErrors = 9;
}

bool loadSettings() {
  loadDefaults();
  // open read/write so the namespace gets created on first boot
  if (!prefs.begin("propbomb", false)) {
    return false;
  }
  String d = prefs.getString("dcode", cfg.disarmCode);
  String a = prefs.getString("acode", cfg.adminCode);
  cfg.countdownSec = prefs.getUInt("cdsec", cfg.countdownSec);
  cfg.maxErrors    = prefs.getUChar("maxe", cfg.maxErrors);
  cfg.penaltySec   = prefs.getUShort("pen", cfg.penaltySec);
  cfg.mode         = prefs.getUChar("mode", cfg.mode);
  cfg.wireStageSec = prefs.getUInt("wsec", cfg.wireStageSec);
  cfg.startOnOpen  = prefs.getUChar("ston", cfg.startOnOpen);
  prefs.end();

  d.toCharArray(cfg.disarmCode, sizeof(cfg.disarmCode));
  a.toCharArray(cfg.adminCode, sizeof(cfg.adminCode));

  clampSettings();
  return true;
}

bool saveSettings() {
  clampSettings();
  if (!prefs.begin("propbomb", false)) {
    return false;
  }
  prefs.putString("dcode", cfg.disarmCode);
  prefs.putString("acode", cfg.adminCode);
  prefs.putUInt("cdsec", cfg.countdownSec);
  prefs.putUChar("maxe", cfg.maxErrors);
  prefs.putUShort("pen", cfg.penaltySec);
  prefs.putUChar("mode", cfg.mode);
  prefs.putUInt("wsec", cfg.wireStageSec);
  prefs.putUChar("ston", cfg.startOnOpen);
  prefs.end();
  return true;
}

// -------------------- STATE --------------------
enum State { STATE_IDLE, STATE_ARMED, STATE_DISARMED, STATE_FAILED, STATE_PROGRAM, STATE_WIRE_STAGE };
State state = STATE_IDLE;
const char* stateName(State s) {
  switch (s) {
    case STATE_IDLE: return "IDLE";
    case STATE_ARMED: return "ARMED";
    case STATE_DISARMED: return "DISARMED";
    case STATE_FAILED: return "FAILED";
    case STATE_PROGRAM: return "PROGRAM";
    case STATE_WIRE_STAGE: return "WIRE_STAGE";
    default: return "?";
  }
}

uint32_t tEnd = 0;
uint32_t tLastBeep = 0;
uint8_t errorsLeft = 0; // kept for log, no auto-explode
String inputBuf;
enum BlinkColor { BLINK_NONE, BLINK_GREEN, BLINK_RED };
BlinkColor blinkColor = BLINK_NONE;
uint32_t blinkEndMs = 0;
bool wireOkInitial=false, wireBad1Initial=false, wireBad2Initial=false;
bool isCut(int pin); // forward
bool inputDisplayActive = false;
uint32_t inputDisplayUntil = 0;
uint8_t inputTargetLen = 0;

bool parseHHMM(const String &s, uint32_t &outSeconds) {
  String clean;
  for (size_t i=0;i<s.length();i++) if (isDigit(s[i])) clean += s[i];
  if (clean.length() == 0 || clean.length() > 4) return false;
  uint32_t v = (uint32_t)clean.toInt();
  uint32_t hr = v / 100;
  uint32_t mi = v % 100;
  if (mi > 59 || hr > 99 || (hr==0 && mi==0)) return false;
  outSeconds = hr * 3600UL + mi * 60UL;
  return true;
}

// program combo *#
char lastKey = 0;
uint32_t lastKeyMs = 0;

// lid
bool lastLid = true;

// -------------------- UTILS --------------------
uint32_t nowMs() { return (uint32_t)millis(); }
void buzzer(bool on) { digitalWrite(BUZZ_PIN, on ? HIGH : LOW); }
void beep(uint16_t ms=150) { buzzer(true); delay(ms); buzzer(false); }

void setLed1(uint8_t r,uint8_t g,uint8_t b){ led1.setPixelColor(0, led1.Color(r,g,b)); led1.show(); }
void setLed2(uint8_t r,uint8_t g,uint8_t b){ led2.setPixelColor(0, led2.Color(r,g,b)); led2.show(); }
void stopBlink(){ blinkColor = BLINK_NONE; blinkEndMs = 0; }
void startBlink(BlinkColor c, uint32_t durMs){
  blinkColor = c;
  blinkEndMs = nowMs() + durMs;
}
bool renderBlink(){
  if (blinkColor == BLINK_NONE) return false;
  uint32_t now = nowMs();
  if (now >= blinkEndMs) { stopBlink(); setLed1(0,0,0); setLed2(0,0,0); return false; }
  bool on = ((now / 300) % 2) == 0;
  if (blinkColor == BLINK_GREEN) {
    if (on) { setLed1(0,255,0); setLed2(0,255,0); }
    else { setLed1(0,0,0); setLed2(0,0,0); }
  } else if (blinkColor == BLINK_RED) {
    if (on) { setLed1(255,0,0); setLed2(255,0,0); }
    else { setLed1(0,0,0); setLed2(0,0,0); }
  }
  return true;
}

void showMMSS(uint32_t seconds) {
  if (seconds > 5999) seconds = 5999;
  uint32_t mm = seconds / 60;
  uint32_t ss = seconds % 60;
  uint16_t v = (uint16_t)(mm * 100 + ss);
  seg.showNumberDecEx(v, 0b01000000, true);
}

void showInputDigits() {
  if (inputBuf.length() == 0) { seg.showNumberDec(0, true); return; }
  uint8_t data[4] = {0,0,0,0};
  size_t len = inputBuf.length();
  if (len > 4) len = 4;
  for (size_t i=0; i<len; i++) {
    data[i] = seg.encodeDigit(inputBuf[i] - '0'); // left to right
  }
  seg.setSegments(data);
}

uint32_t remainingSec() {
  uint32_t now = nowMs();
  if (now >= tEnd) return 0;
  return (tEnd - now) / 1000UL;
}

uint32_t beepIntervalMs(uint32_t rem) {
  if (rem > 120) return 5000;            // ogni 5s finché mancano più di 2 minuti
  if (rem > 60)  return 1000;            // penultimo minuto: 1s
  // ultimo minuto: parte da 500ms e scala verso 120ms
  uint32_t scaled = (500U * rem) / 60U;
  if (scaled < 120) scaled = 120;
  return scaled;
}

void countdownLeds(uint32_t rem, uint32_t total) {
  if (blinkColor != BLINK_NONE) return; // non interferire con blink di stato
  float ratio = (total > 0) ? (1.0f - (float)rem / (float)total) : 1.0f;
  uint8_t r = (uint8_t)(255 * ratio);
  uint8_t g = (uint8_t)(255 * (1.0f - ratio));
  uint32_t blinkMs = (rem > 60) ? 600 : (rem > 30 ? 350 : 180);
  bool on = ((nowMs() / blinkMs) % 2) == 0;
  if (on) { setLed1(r,g,0); }
  else { setLed1(0,0,0); }
}

void testDisplay() {
  seg.clear();
  uint8_t data[4] = {0,0,0,0};
  for (int i=0;i<4;i++){
    memset(data, 0, sizeof(data));
    data[i] = seg.encodeDigit(8); // accende tutte le barre su un digit
    seg.setSegments(data);
    delay(220);
  }
  seg.clear();
}

void testBeep() {
  beep(180);
  delay(120);
  beep(80);
}

void testLed1() {
  setLed1(255,0,0); delay(150);
  setLed1(0,255,0); delay(150);
  setLed1(0,0,255); delay(150);
  setLed1(255,255,255); delay(150);
  setLed1(0,0,0);
}

void testLed2() {
  setLed2(255,0,0); delay(150);
  setLed2(0,255,0); delay(150);
  setLed2(0,0,255); delay(150);
  setLed2(255,255,255); delay(150);
  setLed2(0,0,0);
}

void runStartupTest() {
  testDisplay();
  testLed1();
  testLed2();
  testBeep();
  stopBlink();
}

// -------------------- LID --------------------
bool lidRead(){ return digitalRead(LID_PIN); }
bool shouldStartOnChange(bool prev, bool cur){
  if (cfg.startOnOpen) return (prev == false && cur == true);
  return (prev == true && cur == false);
}

// -------------------- CORE ACTIONS --------------------
void armCountdown() {
  errorsLeft = cfg.maxErrors;
  inputBuf = "";
  tEnd = nowMs() + (cfg.countdownSec * 1000UL);
  state = STATE_ARMED;
   stopBlink();
  setLed2(0,0,0);
  beep(80);
}

void explode() {
  state = STATE_FAILED;
  for (int i=0;i<6;i++){
    setLed1(255,0,0);
    setLed2(255,0,0);
    buzzer(true); delay(120);
    setLed1(0,0,0);
    setLed2(0,0,0);
    buzzer(false); delay(120);
  }
  setLed1(255,0,0);
  setLed2(255,0,0);
  startBlink(BLINK_RED, 120000); // blink red for 2 minutes
}

void disarmSuccess() {
  state = STATE_DISARMED;
  setLed1(0,255,0);
  setLed2(0,255,0);
  for (int i=0;i<3;i++){ beep(80); delay(60); }
  startBlink(BLINK_GREEN, 120000); // blink green for 2 minutes
}

void enterWireStage() {
  state = STATE_WIRE_STAGE;
  inputBuf = "";
  tEnd = nowMs() + (cfg.wireStageSec * 1000UL);
  setLed2(255,255,255); // LED2 bianco per wire stage
  stopBlink();
  beep(120);
  // snapshot initial wire states to detect change
  delay(20);
  wireOkInitial = isCut(WIRE_OK_PIN);
  wireBad1Initial = isCut(WIRE_BAD1_PIN);
  wireBad2Initial = isCut(WIRE_BAD2_PIN);
}

// -------------------- PROGRAM MODE (tastiera) --------------------
enum ProgStep { P_DISARM, P_ADMIN, P_TIME, P_ERRS, P_PEN, P_MODE, P_WTIME, P_STON, P_SAVE };
ProgStep pStep = P_DISARM;
String pTemp = "";
uint8_t progStepIndex = 0; // 0-based for indicator
ProgStep lastRenderStep = P_DISARM;
size_t lastRenderTempLen = 0;

void showProgPrompt(uint8_t code, int value) {
  int v = ((int)code * 100) + (value % 100);
  seg.showNumberDec(v, true);
}

void showStepIndicator(uint8_t stepIdx) {
  // light a single segment progressing through digits (0-3 digits, 0-6 segments each)
  // segments bit order for TM1637: 0b0GFEDCBA (A is LSB)
  static const uint8_t segmentOrder[7] = {0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000};
  uint8_t digit = stepIdx / 7;
  uint8_t segIdx = stepIdx % 7;
  uint8_t data[4] = {0,0,0,0};
  if (digit < 4) data[digit] = segmentOrder[segIdx];
  seg.setSegments(data);
}

// forward declarations
bool isCut(int pin);

bool handleNumericEntry(char k, uint32_t &outVal, uint32_t minV, uint32_t maxV) {
  if (k == NO_KEY) return false;
  if (k == '*') { if (pTemp.length()) pTemp.remove(pTemp.length()-1); }
  else if (k >= '0' && k <= '9') { if (pTemp.length() < 9) pTemp += k; }
  else if (k == 'A') {
    if (!pTemp.length()) return false;
    uint32_t v = (uint32_t)pTemp.toInt();
    if (v < minV || v > maxV) { beep(20); delay(40); beep(20); return false; }
    outVal = v; pTemp = ""; return true;
  } else if (k == 'B') { state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); }
  return false;
}

bool handleCodeEntry(char k, char* outCode, size_t maxLen) {
  if (k == NO_KEY) return false;
  if (k == '*') { if (pTemp.length()) pTemp.remove(pTemp.length()-1); }
  else if (k >= '0' && k <= '9') { if (pTemp.length() < (int)maxLen-1) pTemp += k; }
  else if (k == 'A') {
    if (pTemp.length() < 3) { beep(20); delay(40); beep(20); return false; }
    pTemp.toCharArray(outCode, maxLen); pTemp = ""; return true;
  } else if (k == 'B') { state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); }
  return false;
}

void enterProgram() {
  state = STATE_PROGRAM;
  pStep = P_DISARM;
  progStepIndex = 0;
  lastRenderStep = P_DISARM;
  lastRenderTempLen = 0;
  pTemp = "";
  inputBuf = "";
  setLed1(0,0,20);
  setLed2(0,0,20);
  beep(60);
  // blink all digits then clear
  seg.showNumberDec(8888, true); delay(150);
  seg.clear(); delay(150);
  showStepIndicator(progStepIndex);
}

void programLoop(char k) {
  bool stepChanged = (pStep != lastRenderStep);
  bool inputChanged = (pTemp.length() != lastRenderTempLen) || (k != NO_KEY);

  if (stepChanged || inputChanged) {
    if (pStep == P_DISARM || pStep == P_ADMIN) {
      if (pTemp.length()) seg.showNumberDec((int)pTemp.toInt(), false);
      else showStepIndicator(progStepIndex);
    } else if (pStep == P_TIME || pStep == P_WTIME) {
      if (pTemp.length()) {
        uint16_t val = (uint16_t)pTemp.toInt();
        seg.showNumberDec(val, true); // HHMM
      } else showStepIndicator(progStepIndex);
    } else {
      showStepIndicator(progStepIndex);
    }
    lastRenderStep = pStep;
    lastRenderTempLen = pTemp.length();
  }

  switch (pStep) {
    case P_DISARM:
      showProgPrompt(1, (int)strlen(cfg.disarmCode));
      if (handleCodeEntry(k, cfg.disarmCode, sizeof(cfg.disarmCode))) pStep = P_ADMIN;
      break;

    case P_ADMIN:
      showProgPrompt(2, (int)strlen(cfg.adminCode));
      if (handleCodeEntry(k, cfg.adminCode, sizeof(cfg.adminCode))) { pStep = P_TIME; progStepIndex++; }
      break;

    case P_TIME: {
      // input as HHMM (00-99 hours, 00-59 minutes)
      showProgPrompt(3, 0);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 0, 9959)) {
        uint32_t hours = v / 100;
        uint32_t mins = v % 100;
        if (mins > 59 || hours > 99 || (hours==0 && mins==0)) { beep(20); delay(40); beep(20); break; }
        cfg.countdownSec = (hours * 3600UL) + (mins * 60UL);
        pStep = P_ERRS; progStepIndex++;
      }
    } break;

    case P_ERRS: {
      showProgPrompt(4, cfg.maxErrors);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 0, 9)) { cfg.maxErrors = (uint8_t)v; pStep = P_PEN; progStepIndex++; }
    } break;

    case P_PEN: {
      showProgPrompt(5, cfg.penaltySec % 100);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 0, 300)) { cfg.penaltySec = (uint16_t)v; pStep = P_MODE; progStepIndex++; }
    } break;

    case P_MODE: {
      showProgPrompt(6, cfg.mode);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 1, 2)) { cfg.mode = (uint8_t)v; pStep = P_WTIME; progStepIndex++; }
    } break;

    case P_WTIME: {
      // input as HHMM (00-99 hours, 00-59 minutes)
      showProgPrompt(7, 0);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 0, 9959)) {
        uint32_t hours = v / 100;
        uint32_t mins = v % 100;
        if (mins > 59 || hours > 99 || (hours==0 && mins==0)) { beep(20); delay(40); beep(20); break; }
        cfg.wireStageSec = (hours * 3600UL) + (mins * 60UL);
        pStep = P_STON; progStepIndex++;
      }
    } break;

    case P_STON: {
      showProgPrompt(8, cfg.startOnOpen);
      uint32_t v=0;
      if (handleNumericEntry(k, v, 0, 1)) { cfg.startOnOpen = (uint8_t)v; pStep = P_SAVE; progStepIndex++; }
    } break;

    case P_SAVE:
      showProgPrompt(99, 0);
      if (k == 'A') { saveSettings(); beep(80); state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); progStepIndex = 0; }
      else if (k == 'B') { loadSettings(); state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); progStepIndex = 0; }
      break;
  }
}

// -------------------- CODE INPUT (ARMED) --------------------
bool checkCode(const String &buf, const char* code) { return buf.equals(String(code)); }

void handleCodes(char k) {
  if (k == NO_KEY) return;

  if (k == '*') { inputBuf = ""; return; }

  if (k >= '0' && k <= '9') {
    inputBuf += k;

    size_t needLen = max(strlen(cfg.disarmCode), strlen(cfg.adminCode));
    inputTargetLen = (uint8_t)needLen;
    inputDisplayActive = true;
    inputDisplayUntil = nowMs() + 5000;
    showInputDigits();

    if (inputBuf.length() >= needLen) {
      if (checkCode(inputBuf, cfg.adminCode)) {
        disarmSuccess(); // admin stop sempre
      } else if (checkCode(inputBuf, cfg.disarmCode)) {
        if (cfg.mode == 1) {
          disarmSuccess();
        } else {
          enterWireStage();
        }
      } else {
        uint32_t penMs = (uint32_t)cfg.penaltySec * 1000UL;
        uint32_t now = nowMs();
        if (tEnd > now + penMs) {
          tEnd -= penMs;
        } else {
          tEnd = now + 1000; // lascia almeno 1s per nuovi tentativi
        }
        for (int i=0;i<2;i++){ beep(30); delay(40); }
      }
      inputBuf = "";
      inputDisplayActive = false;
      inputTargetLen = 0;
    }
  }
}

// -------------------- WIRE STAGE --------------------
bool isCut(int pin) { return digitalRead(pin) == HIGH; }

void wireStageLoop() {
  uint32_t rem = remainingSec();
  showMMSS(rem);
  setLed2(255,255,255); // LED2 fisso bianco durante wire stage

  // regola fissa (come richiesto)
  if (!wireOkInitial && isCut(WIRE_OK_PIN)) { disarmSuccess(); return; }
  if ((!wireBad1Initial && isCut(WIRE_BAD1_PIN)) || (!wireBad2Initial && isCut(WIRE_BAD2_PIN))) { explode(); return; }

  if (rem == 0) explode();
}

// -------------------- WEB UI --------------------
String htmlEscape(const String &s){
  String o = s;
  o.replace("&","&amp;"); o.replace("<","&lt;"); o.replace(">","&gt;"); o.replace("\"","&quot;");
  return o;
}

String page() {
  String d = htmlEscape(String(cfg.disarmCode));
  String a = htmlEscape(String(cfg.adminCode));

  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Config</title>";
  h += "<style>";
  h += ":root{--bg:#071425;--card:#0f1e32cc;--accent:#1d7cf2;--accent2:#19b6ff;--text:#e9f1ff;--muted:#9db3d3;--shadow:0 12px 30px rgba(0,0,0,0.35);}*{box-sizing:border-box;}body{margin:0;font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:radial-gradient(circle at 20% 20%,rgba(33,107,255,0.18),transparent 22%),radial-gradient(circle at 80% 10%,rgba(33,107,255,0.18),transparent 18%),linear-gradient(135deg,var(--bg),#0b1c34);color:var(--text);}header{position:sticky;top:0;z-index:9;padding:16px 18px 10px;background:linear-gradient(180deg,rgba(7,20,37,0.95),rgba(7,20,37,0.75));backdrop-filter:blur(10px);display:flex;flex-wrap:wrap;align-items:center;gap:12px;box-shadow:var(--shadow);}h1{margin:0;font-size:20px;letter-spacing:0.5px;}#status{padding:6px 12px;border-radius:999px;font-weight:600;font-size:14px;color:var(--text);background:rgba(255,255,255,0.08);border:1px solid rgba(255,255,255,0.12);}main{padding:14px 16px 22px;max-width:1100px;margin:0 auto;}form.card{background:var(--card);border:1px solid rgba(255,255,255,0.08);border-radius:14px;padding:14px 14px 10px;box-shadow:var(--shadow);}fieldset{border:none;margin:0;padding:0;}legend{font-size:14px;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:0.06em;margin-bottom:8px;}label{display:block;font-size:13px;color:var(--muted);margin:10px 0 4px;}input,select{width:100%;padding:11px 12px;border-radius:10px;border:1px solid rgba(255,255,255,0.12);background:rgba(255,255,255,0.04);color:var(--text);font-size:15px;}input:focus,select:focus{outline:2px solid var(--accent);}button{cursor:pointer;border:none;border-radius:12px;padding:11px 12px;font-weight:700;font-size:15px;color:#fff;background:linear-gradient(135deg,var(--accent),var(--accent2));box-shadow:0 10px 20px rgba(29,124,242,0.35);transition:transform 120ms ease,box-shadow 120ms ease;}button:active{transform:translateY(1px);box-shadow:0 8px 14px rgba(29,124,242,0.25);}small{color:var(--muted);} .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px;} .actions{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;} .card h3{margin:0 0 8px;font-size:16px;} .pill{display:inline-flex;align-items:center;gap:6px;padding:6px 10px;border-radius:999px;background:rgba(255,255,255,0.06);color:var(--muted);font-size:13px;} .divider{height:1px;border:none;margin:14px 0;background:rgba(255,255,255,0.08);} .top-actions{display:flex;gap:10px;flex-wrap:wrap;margin-left:auto;} .inline-actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px;} </style>";
  h += "</head><body>";

  h += "<header>";
  h += "<div><h1>Config</h1><div class='pill'>Hotspot <b>"+apSsid+"</b></div><div class='pill'>IP <b>192.168.0.1</b></div></div>";
  h += "<div class='top-actions'>";
  h += "<form method='POST' action='/arm'><button type='submit'>ARM</button></form>";
  h += "<form method='POST' action='/idle'><button type='submit' style='background:linear-gradient(135deg,#6c7686,#3f4654);box-shadow:0 10px 20px rgba(0,0,0,0.25);'>IDLE</button></form>";
  h += "</div>";
  h += "<div id='status'>Stato: ";
  h += (state==STATE_IDLE?"IDLE":state==STATE_ARMED?"ARMED":state==STATE_WIRE_STAGE?"WIRE_STAGE":state==STATE_DISARMED?"DISARMED":state==STATE_FAILED?"FAILED":"PROGRAM");
  h += "</div>";
  h += "</header>";

  h += "<main>";
  h += "<div class='grid'>";

  h += "<form class='card' method='POST' action='/save'>";
  h += "<h3>Impostazioni rapide</h3>";
  h += "<fieldset><legend>Codici</legend>";
  h += "<label>Codice disinnesco</label><input name='dcode' value='"+d+"' minlength='3' maxlength='11' inputmode='numeric'>";
  h += "<label>Codice admin (stop)</label><input name='acode' value='"+a+"' minlength='3' maxlength='11' inputmode='numeric'>";
  h += "</fieldset>";
  h += "<fieldset><legend>Countdown</legend>";
  uint32_t cHr = cfg.countdownSec / 3600;
  uint32_t cMin = (cfg.countdownSec % 3600) / 60;
  char cbuf[8]; snprintf(cbuf, sizeof(cbuf), "%02u%02u", (unsigned)cHr, (unsigned)cMin);
  h += "<label>Durata (HHMM)</label>";
  h += "<input name='cdsec' type='text' pattern='[0-9][0-9][0-5][0-9]' maxlength='4' value='"+String(cbuf)+"' title='HHMM' inputmode='numeric'>";
  h += "<label>Penalità per errore (s)</label>";
  h += "<input name='pen' type='number' min='0' max='300' value='"+String(cfg.penaltySec)+"'>";
  h += "</fieldset>";
  h += "<fieldset><legend>Modalità</legend>";
  h += "<label>Modalità gioco</label>";
  h += "<select name='mode'>";
  h += "<option value='1'"; h += (cfg.mode==1?" selected":""); h += ">1 · Solo codice</option>";
  h += "<option value='2'"; h += (cfg.mode==2?" selected":""); h += ">2 · Codice + Wire-Stage</option>";
  h += "</select>";
  uint32_t wHr = cfg.wireStageSec / 3600;
  uint32_t wMin = (cfg.wireStageSec % 3600) / 60;
  char wbuf[8]; snprintf(wbuf, sizeof(wbuf), "%02u%02u", (unsigned)wHr, (unsigned)wMin);
  h += "<label>Durata Wire-Stage (HHMM)</label>";
  h += "<input name='wsec' type='text' pattern='[0-9][0-9][0-5][0-9]' maxlength='4' value='"+String(wbuf)+"' title='HHMM' inputmode='numeric'>";
  h += "<label>Trigger valigetta</label>";
  h += "<select name='ston'>";
  h += "<option value='1'"; h += (cfg.startOnOpen==1?" selected":""); h += ">Su apertura contatto</option>";
  h += "<option value='0'"; h += (cfg.startOnOpen==0?" selected":""); h += ">Su chiusura contatto</option>";
  h += "</select>";
  h += "</fieldset>";
  h += "<div class='inline-actions'><button type='submit'>Salva</button></div>";
  h += "</form>";

  h += "<div class='card'><h3>Test rapidi</h3>";
  h += "<div class='actions'>";
  h += "<form method='POST' action='/test_display'><button>Test Display</button></form>";
  h += "<form method='POST' action='/test_beep'><button>Test Beep</button></form>";
  h += "<form method='POST' action='/test_led1'><button>LED1 (WS2812)</button></form>";
  h += "<form method='POST' action='/test_led2'><button>LED2 (WS2812)</button></form>";
  h += "</div>";
  h += "</div>";

  h += "</div>"; // grid
  h += "<div class='divider'></div>";
  h += "<small>Hotspot: <b>"+apSsid+"</b> · Password: <b>"+apPass+"</b></small>";
  h += "</main>";
  h += "</body></html>";
  return h;
}

bool isCodeDigits(const String &s){
  if (s.length() < 3 || s.length() > 11) return false;
  for (size_t i=0;i<s.length();i++) if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

void webSetup() {
  server.on("/", HTTP_GET, [](){
    server.send(200, "text/html", page());
  });

  server.on("/save", HTTP_POST, [](){
    String dcode = server.arg("dcode");
    String acode = server.arg("acode");
    uint32_t cdsec = cfg.countdownSec;
    if (server.hasArg("cdsec")) {
      String s = server.arg("cdsec");
      if (!parseHHMM(s, cdsec)) { server.send(400, "text/plain", "Countdown HHMM non valido"); return; }
    }
    uint32_t wsec  = cfg.wireStageSec;
    if (server.hasArg("wsec")) {
      String s = server.arg("wsec");
      if (!parseHHMM(s, wsec)) { server.send(400, "text/plain", "Wire-stage HHMM non valido"); return; }
    }
    uint16_t pen   = (uint16_t) server.arg("pen").toInt();
    uint8_t mode   = (uint8_t)  server.arg("mode").toInt();
    if (mode < 1 || mode > 2) mode = cfg.mode;
    uint8_t ston   = (uint8_t)  server.arg("ston").toInt();

    if (!isCodeDigits(dcode) || !isCodeDigits(acode)) {
      server.send(400, "text/plain", "Codici non validi (solo numeri, 3-11 cifre).");
      return;
    }

    dcode.toCharArray(cfg.disarmCode, sizeof(cfg.disarmCode));
    acode.toCharArray(cfg.adminCode, sizeof(cfg.adminCode));
    cfg.countdownSec = cdsec;
    cfg.wireStageSec = wsec;
    cfg.penaltySec = pen;
    cfg.mode = mode;
    cfg.startOnOpen = ston;

    if (!saveSettings()) { server.send(500, "text/plain", "Salvataggio fallito"); return; }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/arm", HTTP_POST, [](){
    if (state == STATE_IDLE) armCountdown();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/idle", HTTP_POST, [](){
    state = STATE_IDLE;
    stopBlink();
    setLed1(0,0,0); setLed2(0,0,0);
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/test_display", HTTP_POST, [](){
    testDisplay();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/test_beep", HTTP_POST, [](){
    testBeep();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/test_led1", HTTP_POST, [](){
    testLed1();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.on("/test_led2", HTTP_POST, [](){
    testLed2();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
}

void wifiStartAP() {
  // SSID fisso richiesto
  apSsid = "PROPDETONATORE1";

  // password semplice ma non “aperta” (puoi cambiarla)
  apPass = "pwdpropdetonatore1";

  IPAddress localIp(192, 168, 0, 1);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIp, gateway, subnet); // DHCP partirà da 192.168.0.2
  WiFi.softAP(apSsid.c_str(), apPass.c_str());
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  pinMode(BUZZ_PIN, OUTPUT);
  buzzer(false);

  pinMode(LID_PIN, INPUT_PULLUP);
  pinMode(WIRE_OK_PIN, INPUT_PULLUP);
  pinMode(WIRE_BAD1_PIN, INPUT_PULLUP);
  pinMode(WIRE_BAD2_PIN, INPUT_PULLUP);

  seg.setBrightness(7);
  seg.clear();

  led1.begin(); led1.show();
  led2.begin(); led2.show();

  loadSettings();

  state = STATE_IDLE;
  setLed1(0,0,0);
  setLed2(0,0,0);

  lastLid = lidRead();

  // WiFi AP + Web config
  wifiStartAP();
  webSetup();
  runStartupTest();
}

void loop() {
  // gestisci web server
  server.handleClient();
  bool blinkActive = renderBlink();

  // contatto valigetta (start)
  bool lid = lidRead();
  if (lid != lastLid) {
    delay(15);
    lid = lidRead();
      if (lid != lastLid) {
        if (state == STATE_IDLE && shouldStartOnChange(lastLid, lid)) armCountdown();
        lastLid = lid;
      }
  }

  char k = keypad.getKey();

  // combo "*#" entro 600ms -> PROGRAM (da IDLE)
  if (k != NO_KEY) {
    uint32_t t = nowMs();
    if (lastKey == '*' && k == '#' && (t - lastKeyMs) < 600 && state == STATE_IDLE) {
      enterProgram();
      lastKey = 0;
    } else {
      lastKey = k;
      lastKeyMs = t;
    }
  }

  switch (state) {
    case STATE_IDLE:
      seg.clear();
      break;

    case STATE_PROGRAM:
      programLoop(k);
      break;

    case STATE_ARMED: {
      uint32_t rem = remainingSec();
      if (!inputDisplayActive) showMMSS(rem);

      uint32_t interval = beepIntervalMs(rem);
      if (nowMs() - tLastBeep >= interval) {
        tLastBeep = nowMs();
        beep(120);
      }

      if (!blinkActive) {
        countdownLeds(rem, cfg.countdownSec);
        setLed2(0,0,0);
      }
      handleCodes(k);

      if (inputDisplayActive) {
        showInputDigits();
        if (nowMs() > inputDisplayUntil && inputBuf.length() < inputTargetLen) {
          inputBuf = "";
          inputDisplayActive = false;
          inputTargetLen = 0;
          showMMSS(rem);
        }
      }

      if (rem == 0 && state == STATE_ARMED) explode();
    } break;

    case STATE_WIRE_STAGE: {
      uint32_t rem = remainingSec();
      showMMSS(rem);

      uint32_t interval = beepIntervalMs(rem);
      if (nowMs() - tLastBeep >= interval) {
        tLastBeep = nowMs();
        beep(120);
      }

      if (!blinkActive) {
        // LED2 fisso bianco solo se non in lampeggio di stato
        setLed2(255,255,255);

        // LED1: blink colore in base al tempo wire stage
        bool on = ((nowMs() / interval) % 2) == 0;
        if (on) {
          if (rem > 120) setLed1(0,255,0);          // verde
          else if (rem > 60) setLed1(255,200,0);    // giallo
          else setLed1(255,0,0);                    // rosso
        } else {
          setLed1(0,0,0);
        }
      }

      wireStageLoop();
    } break;

    case STATE_DISARMED:
      showMMSS(0);
      if (k == 'D') { state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); stopBlink(); }
      break;

    case STATE_FAILED:
      showMMSS(0);
      if (k == 'D') { state = STATE_IDLE; setLed1(0,0,0); setLed2(0,0,0); stopBlink(); }
      break;
  }
}
