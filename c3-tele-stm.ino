#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- KONFIGURASI PIN UART ESP32-C3 (TERHUBUNG KE STM32F4 BLACK PILL) ---
#define RX_PIN 20  // Terhubung ke PA2 (TX2) STM32F4
#define TX_PIN 21  // Terhubung ke PA3 (RX2) STM32F4

// --- MAC ADDRESS TRANSMITTER & DISPLAY ---
uint8_t transmitterMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
uint8_t displayMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
Preferences preferences;
AsyncWebServer server(80);

// --- VARIABEL EFISIENSI & TIMEOUT WIFI (5 MENIT) ---
unsigned long lastActivityTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 300000; 
bool isWifiActive = true;

// --- STATUS KONEKSI UART & SINKRONISASI ---
unsigned long lastUartRxTime = 0;
const unsigned long UART_TIMEOUT_MS = 2000;
bool isEcuConnected = false;
bool initialSyncDone = false;
unsigned long lastSyncRetryTime = 0;
uint8_t syncRetryCount = 0;

// --- DATA LOGGER CONFIGURATION (RAM RING BUFFER) ---
#define MAX_LOG_ENTRIES 800

struct LogEntry {
  uint32_t timestampMs;
  uint16_t rpm;
  uint8_t  tps;
  int16_t  degree10;
  int16_t  temp10;
  uint16_t battVolt10;
};

LogEntry logBuffer[MAX_LOG_ENTRIES];
uint16_t logHead = 0;
uint16_t logCount = 0;
bool bufferOverflowed = false; 
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

// --- DATA BAWAAN DEFAULT ---
const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

const uint16_t defaultDwellPerRPM[NUM_RPM_POINTS] = {
  3500, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200
};

struct CustomTuning {
  int16_t mapData[NUM_RPM_POINTS][NUM_TPS_POINTS];
  uint16_t dwellData[NUM_RPM_POINTS]; 
  uint16_t rpmLimit;
};
CustomTuning customMapSlots[5];
uint8_t currentActiveSlot = 0; 

// --- STRUKTUR PAKET KOMUNIKASI ---
struct __attribute__((packed)) TelemetryData {
  uint16_t header;     // 0xAA55
  uint16_t rpm;
  uint8_t  tps;        
  int16_t  degree10;   
  int16_t  temp10;     
  uint16_t battVolt10; 
  uint8_t  mode;
  uint8_t  fuel;
  uint8_t  state;
  uint8_t  ackStatus;  
  uint16_t faultCode;  
  uint16_t crc16;
};

struct __attribute__((packed)) CommandData {
  uint16_t header;     // 0xCC55
  uint8_t requestedMode;
  uint16_t crc16;
};

struct __attribute__((packed)) CommandPacketToECU {
  uint16_t header;                                  // 0x55CC
  uint8_t  cmdType;                                 // 0x01: Mode, 0x02: RAM Map, 0x03: Save EEPROM
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; 
  uint16_t dwellData[NUM_RPM_POINTS];               
  uint16_t rpmLimit;                                
  uint16_t crc16;                                   
};

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryData currentTelemetry = {0xAA55, 0, 0, 100, 300, 126, 0, 0, 0, 0, 0, 0};

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021; else crc <<= 1;
    }
  }
  return crc;
}

void pushToLog(const TelemetryData *pkt) {
  if (pkt->rpm < 400) return;
  portENTER_CRITICAL(&logMux);
  logBuffer[logHead] = (LogEntry){
    .timestampMs = millis(),
    .rpm = pkt->rpm,
    .tps = pkt->tps,
    .degree10 = pkt->degree10,
    .temp10 = pkt->temp10,
    .battVolt10 = pkt->battVolt10
  };
  logHead = (logHead + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) {
    logCount++;
  } else {
    bufferOverflowed = true;
  }
  portEXIT_CRITICAL(&logMux);
}

void sendCommandToECU(uint8_t cmdType, uint8_t slotOrMode) {
  CommandPacketToECU cmd;
  cmd.header = 0x55CC;
  cmd.cmdType = cmdType;
  cmd.slotOrMode = slotOrMode;
  
  if ((cmdType == 0x02 || cmdType == 0x03) && slotOrMode < 5) {
    memcpy(cmd.mapData, customMapSlots[slotOrMode].mapData, sizeof(cmd.mapData));
    memcpy(cmd.dwellData, customMapSlots[slotOrMode].dwellData, sizeof(cmd.dwellData));
    cmd.rpmLimit = customMapSlots[slotOrMode].rpmLimit;
  } else {
    memset(cmd.mapData, 0, sizeof(cmd.mapData)); 
    memcpy(cmd.dwellData, defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
    cmd.rpmLimit = 12500;
  }
  
  cmd.crc16 = calculateCRC16((uint8_t*)&cmd, sizeof(CommandPacketToECU) - sizeof(uint16_t));
  Serial1.write((uint8_t*)&cmd, sizeof(CommandPacketToECU));
}

void parseUART() {
  static uint8_t rxBuffer[sizeof(TelemetryData)];
  static size_t rxIndex = 0;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (rxIndex == 0) { if (c == 0xAA) rxBuffer[rxIndex++] = c; continue; } 
    else if (rxIndex == 1) {
      if (c == 0x55) rxBuffer[rxIndex++] = c; else rxIndex = (c == 0xAA) ? 1 : 0; continue;
    }
    rxBuffer[rxIndex++] = c;

    if (rxIndex >= sizeof(TelemetryData)) {
      TelemetryData *pkt = (TelemetryData*)rxBuffer;
      if (pkt->crc16 == calculateCRC16(rxBuffer, sizeof(TelemetryData) - sizeof(uint16_t))) {
        portENTER_CRITICAL(&dataMux);
        memcpy(&currentTelemetry, pkt, sizeof(TelemetryData));
        portEXIT_CRITICAL(&dataMux);
        
        lastUartRxTime = millis();
        if (!isEcuConnected) {
          isEcuConnected = true;
          initialSyncDone = false;
          syncRetryCount = 0;
          lastSyncRetryTime = 0;
        }
        
        if (!initialSyncDone && syncRetryCount > 0 && pkt->ackStatus == 1) {
          initialSyncDone = true;
        }
        
        pushToLog(pkt); 
        esp_now_send(displayMacAddress, rxBuffer, sizeof(TelemetryData));
      }
      rxIndex = 0;
    }
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}
void OnEspNowRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
#else
void OnEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
void OnEspNowRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(CommandData)) {
    CommandData *cmd = (CommandData*)incomingData;
    uint16_t calcCrc = calculateCRC16(incomingData, sizeof(CommandData) - sizeof(uint16_t));
    if (cmd->header == 0xCC55 && cmd->crc16 == calcCrc) {
      sendCommandToECU(0x01, cmd->requestedMode); 
    }
  }
}

void loadMapFromNVS() {
  preferences.begin("ecu_tuning", false);
  currentActiveSlot = preferences.getUChar("activeSlot", 0);
  if (currentActiveSlot > 4) currentActiveSlot = 0;

  if (preferences.getBytesLength("customMaps") == sizeof(customMapSlots)) {
    preferences.getBytes("customMaps", &customMapSlots, sizeof(customMapSlots));
  } else {
    for (int s = 0; s < 5; s++) {
      customMapSlots[s].rpmLimit = 12500;
      memcpy(customMapSlots[s].dwellData, defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
      memcpy(customMapSlots[s].mapData, mapDaily3DBase, sizeof(mapDaily3DBase));
    }
  }
  preferences.end();
}

void saveMapToNVS() {
  preferences.begin("ecu_tuning", false);
  preferences.putUChar("activeSlot", currentActiveSlot);
  preferences.putBytes("customMaps", &customMapSlots, sizeof(customMapSlots));
  preferences.end();
}

const char* htmlUI = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TEAM PATAS KUDUS - ECU TUNER & LOGGER</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700;900&family=Rajdhani:wght@500;600;700&display=swap');
  :root {
    --bg-main: #070709;
    --bg-card: rgba(18, 18, 24, 0.75);
    --border-color: rgba(0, 255, 135, 0.2);
    --neon-green: #00ff87;
    --neon-cyan: #00e5ff;
    --neon-pink: #ff0055;
    --neon-yellow: #ffea00;
    --text-main: #f0f0f5;
    --text-muted: #8a8a9e;
  }
  body { font-family: 'Rajdhani', sans-serif; background: var(--bg-main); color: var(--text-main); margin: 0; padding: 0; overflow-x: hidden; }
  
  /* Marquee Top */
  .marquee-box { background: linear-gradient(90deg, #ff0055, #00ff87, #00e5ff, #ff0055); padding: 2px 0; box-shadow: 0 0 15px rgba(0,255,135,0.3); }
  .marquee-inner { background: #000; overflow: hidden; white-space: nowrap; padding: 5px 0; }
  .marquee-text { display: inline-block; font-family: 'Orbitron', sans-serif; font-size: 12px; font-weight: 700; color: var(--neon-green); text-transform: uppercase; animation: marquee 18s linear infinite; letter-spacing: 1px; }
  @keyframes marquee { 0% { transform: translateX(100%); } 100% { transform: translateX(-100%); } }

  .header { text-align: center; padding: 12px 10px 4px 10px; }
  .header h1 { font-family: 'Orbitron', sans-serif; font-size: 22px; margin: 0; color: #fff; text-shadow: 0 0 12px rgba(0,255,135,0.6); letter-spacing: 2px; }

  .container { max-width: 650px; margin: 0 auto; padding: 10px; }

  /* Status Bar Modern */
  .status-bar { display: flex; justify-content: space-between; align-items: center; background: rgba(12, 12, 18, 0.9); padding: 10px 14px; border-radius: 8px; margin-bottom: 12px; font-family: 'Orbitron', sans-serif; font-size: 11px; border: 1px solid rgba(255,255,255,0.08); box-shadow: inset 0 0 10px rgba(0,0,0,0.8); }
  .ecu-online { color: var(--neon-green); font-weight: 700; text-shadow: 0 0 8px rgba(0,255,135,0.7); }
  .ecu-offline { color: var(--neon-pink); font-weight: 700; text-shadow: 0 0 8px rgba(255,0,85,0.7); animation: blink 1s infinite; }
  @keyframes blink { 50% { opacity: 0.3; } }

  .fault-banner { background: rgba(255, 0, 85, 0.15); border: 1px solid var(--neon-pink); color: #ff3366; padding: 8px; border-radius: 8px; margin-bottom: 12px; font-family: 'Orbitron', sans-serif; font-size: 11px; text-align: center; display: none; font-weight: bold; box-shadow: 0 0 10px rgba(255,0,85,0.3); }

  /* Navigation Tabs */
  .tab-buttons { display: flex; gap: 6px; margin-bottom: 12px; }
  .tab-btn { flex: 1; background: rgba(20,20,28,0.9); border: 1px solid rgba(255,255,255,0.1); color: var(--text-muted); padding: 10px 4px; font-family: 'Orbitron', sans-serif; font-size: 10px; font-weight: 700; border-radius: 8px; cursor: pointer; text-align: center; transition: all 0.2s ease; }
  .tab-btn:hover { border-color: var(--neon-cyan); color: #fff; }
  .tab-btn.active { background: linear-gradient(135deg, rgba(0,255,135,0.2), rgba(0,229,255,0.2)); color: var(--neon-green); border-color: var(--neon-green); box-shadow: 0 0 12px rgba(0,255,135,0.25); }

  /* Cards */
  .card { background: var(--bg-card); backdrop-filter: blur(10px); border: 1px solid var(--border-color); border-radius: 12px; padding: 14px; margin-bottom: 12px; box-shadow: 0 8px 32px rgba(0,0,0,0.4); }
  .controls-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  label { font-family: 'Orbitron', sans-serif; font-size: 10px; color: var(--text-muted); display: block; margin-bottom: 6px; letter-spacing: 1px; }
  select, input[type=number].cfg { width: 100%; box-sizing: border-box; background: #040406; color: var(--neon-green); border: 1px solid rgba(255,255,255,0.12); padding: 9px; font-family: 'Orbitron', sans-serif; font-size: 12px; border-radius: 8px; outline: none; transition: 0.2s; }
  select:focus, input.cfg:focus { border-color: var(--neon-green); box-shadow: 0 0 8px rgba(0,255,135,0.3); }

  /* RACING LIVE GAUGES & BARS */
  .gauge-container { display: flex; flex-direction: column; align-items: center; justify-content: center; position: relative; margin: 10px 0; }
  .circular-gauge { position: relative; width: 180px; height: 180px; border-radius: 50%; background: conic-gradient(#00ff87 0deg, #00e5ff 0deg, #161622 0deg); display: flex; align-items: center; justify-content: center; box-shadow: 0 0 20px rgba(0,255,135,0.2), inset 0 0 20px rgba(0,0,0,0.9); transition: background 0.1s linear; }
  .circular-gauge::before { content: ''; position: absolute; width: 145px; height: 145px; background: #0b0b10; border-radius: 50%; box-shadow: inset 0 0 10px rgba(0,0,0,0.8); }
  .gauge-inner-content { position: relative; z-index: 2; text-align: center; }
  .gauge-title { font-family: 'Orbitron', sans-serif; font-size: 10px; color: var(--text-muted); letter-spacing: 2px; }
  .gauge-value { font-family: 'Orbitron', sans-serif; font-size: 28px; font-weight: 900; color: #fff; text-shadow: 0 0 10px rgba(255,255,255,0.5); }
  .gauge-unit { font-family: 'Orbitron', sans-serif; font-size: 10px; color: var(--neon-green); font-weight: bold; }

  /* Racing Bar Styles */
  .racing-bar-wrapper { background: rgba(10,10,14,0.9); border: 1px solid rgba(255,255,255,0.08); border-radius: 10px; padding: 10px 14px; margin-bottom: 8px; }
  .racing-bar-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; font-family: 'Orbitron', sans-serif; font-size: 11px; }
  .racing-bar-track { background: #161622; height: 10px; border-radius: 5px; overflow: hidden; position: relative; box-shadow: inset 0 2px 4px rgba(0,0,0,0.8); }
  .racing-bar-fill { height: 100%; width: 0%; border-radius: 5px; transition: width 0.1s ease-out; box-shadow: 0 0 10px currentColor; }

  /* Charts & Tables */
  .chart-card { background: rgba(5,5,8,0.9); border: 1px solid rgba(255,255,255,0.08); border-radius: 10px; padding: 12px; margin-bottom: 12px; box-shadow: inset 0 0 15px rgba(0,0,0,0.5); }
  .chart-title { font-family: 'Orbitron', sans-serif; font-size: 11px; color: var(--neon-cyan); text-align: center; margin-bottom: 8px; letter-spacing: 1px; }
  
  .table-responsive { overflow-x: auto; margin-bottom: 15px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.08); max-height: 350px; background: rgba(8,8,12,0.9); }
  table { width: 100%; border-collapse: collapse; }
  th, td { border: 1px solid rgba(255,255,255,0.05); padding: 8px 6px; text-align: center; font-size: 13px; }
  th { background: #12121a; color: var(--neon-cyan); font-family: 'Orbitron', sans-serif; font-size: 10px; position: sticky; top: 0; z-index: 2; letter-spacing: 1px; }
  
  input.cell { width: 44px; background: #020204; color: var(--neon-green); border: 1px solid rgba(255,255,255,0.1); text-align: center; padding: 6px 2px; font-family: 'Rajdhani', sans-serif; font-weight: 700; font-size: 14px; border-radius: 6px; transition: 0.2s; outline: none; }
  input.cell:focus { border-color: var(--neon-cyan); box-shadow: 0 0 8px rgba(0,229,255,0.4); }
  input.cell.cell-drop { background: rgba(255,0,85,0.2) !important; color: #ff3366 !important; border-color: var(--neon-pink) !important; box-shadow: 0 0 8px var(--neon-pink); }
  input.cell.cell-active { background: var(--neon-green) !important; color: #000 !important; font-weight: 900; box-shadow: 0 0 12px var(--neon-green); }
  input.dwell-cell { color: var(--neon-cyan); width: 80px; }

  /* Quick Adjustment Bar */
  .quick-adj-bar { display: flex; gap: 8px; margin-bottom: 12px; background: rgba(16,16,24,0.9); padding: 10px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.08); align-items: center; justify-content: space-around; }
  .btn-adj { background: #1a1a26; border: 1px solid var(--neon-cyan); color: var(--neon-cyan); font-family: 'Orbitron', sans-serif; font-weight: bold; font-size: 12px; padding: 8px 14px; border-radius: 6px; cursor: pointer; transition: 0.2s; }
  .btn-adj:active, .btn-adj:hover { background: var(--neon-cyan); color: #000; box-shadow: 0 0 10px rgba(0,229,255,0.5); }

  .tr-alert { background: rgba(255, 0, 85, 0.2) !important; color: #ff3366 !important; font-weight: bold; }
  
  /* Action Buttons */
  .btn-save { background: linear-gradient(135deg, var(--neon-pink), #ff5500); color: #fff; font-family: 'Orbitron', sans-serif; font-weight: 900; border: none; padding: 14px 20px; font-size: 13px; border-radius: 10px; cursor: pointer; width: 100%; letter-spacing: 2px; margin-bottom: 10px; text-transform: uppercase; box-shadow: 0 4px 20px rgba(255,0,85,0.4); transition: 0.2s; }
  .btn-save:hover { filter: brightness(1.1); box-shadow: 0 6px 25px rgba(255,0,85,0.6); }

  .btn-log { background: linear-gradient(135deg, var(--neon-cyan), #0077ff); color: #fff; font-family: 'Orbitron', sans-serif; font-weight: 900; border: none; padding: 12px 20px; font-size: 12px; border-radius: 10px; cursor: pointer; width: 100%; display: block; text-align: center; text-decoration: none; box-sizing: border-box; margin-bottom: 8px; box-shadow: 0 4px 15px rgba(0,229,255,0.3); }
  .btn-clear { background: #1a1a26; color: #ccc; font-family: 'Orbitron', sans-serif; font-weight: 700; border: 1px solid rgba(255,255,255,0.1); padding: 10px 20px; font-size: 11px; border-radius: 10px; cursor: pointer; width: 100%; text-align: center; box-sizing: border-box; transition: 0.2s; }
  .btn-clear:hover { background: #2a2a38; color: #fff; }
</style>
</head><body>

<div class="marquee-box"><div class="marquee-inner"><div class="marquee-text">🏁 TEAM PATAS KUDUS SATRIA CLUB — TCI IGNITION MAP — RACING TELEMETRY SYSTEM — 🏁</div></div></div>
<div class="header"><h1>TEAM PATAS KUDUS</h1></div>
<div class="container">
  
  <div class="status-bar">
    <span>KONEKSI ECU: <span id="ecuStatus" class="ecu-offline">MENGECEK...</span></span>
    <span>RPM: <span id="liveRpmTop" style="color:var(--neon-green); font-weight:bold;">0</span></span>
  </div>

  <div id="faultBanner" class="fault-banner"></div>

  <!-- TAB NAVIGASI UTAMA -->
  <div class="tab-buttons">
    <button class="tab-btn active" id="btnTabLive" onclick="switchMainTab('live')">LIVE MONITOR</button>
    <button class="tab-btn" id="btnTabDegree" onclick="switchMainTab('degree')">MAP 3D</button>
    <button class="tab-btn" id="btnTabDwell" onclick="switchMainTab('dwell')">DWELL</button>
    <button class="tab-btn" id="btnTabLogger" onclick="switchMainTab('logger')">LOGGER</button>
  </div>

  <!-- 1. TAB LIVE MONITORING & RACING GAUGES -->
  <div id="liveSection">
    <!-- Circular RPM Gauge Card -->
    <div class="card" style="border-color: var(--neon-green); text-align: center;">
      <div class="chart-title">ENGINE SPEED GAUGE</div>
      <div class="gauge-container">
        <div class="circular-gauge" id="rpmCircularGauge">
          <div class="gauge-inner-content">
            <div class="gauge-title">RPM</div>
            <div class="gauge-value" id="gaugeRpmVal">0</div>
            <div class="gauge-unit">RPM</div>
          </div>
        </div>
      </div>
    </div>

    <!-- Racing Bars for Parameters -->
    <div class="card">
      <div class="chart-title" style="margin-bottom: 12px;">RACING PARAMETER BARS</div>
      
      <!-- Advance Degree Bar (0 - 45 deg) -->
      <div class="racing-bar-wrapper">
        <div class="racing-bar-header">
          <span style="color: var(--text-muted);">ADVANCE DEGREE</span>
          <span id="barDegVal" style="color: var(--neon-green); font-weight: bold;">0.0°</span>
        </div>
        <div class="racing-bar-track">
          <div class="racing-bar-fill" id="barDegFill" style="background-color: var(--neon-green);"></div>
        </div>
      </div>

      <!-- TPS Position Bar (0 - 100 %) -->
      <div class="racing-bar-wrapper">
        <div class="racing-bar-header">
          <span style="color: var(--text-muted);">TPS POSITION</span>
          <span id="barTpsVal" style="color: var(--neon-cyan); font-weight: bold;">0%</span>
        </div>
        <div class="racing-bar-track">
          <div class="racing-bar-fill" id="barTpsFill" style="background-color: var(--neon-cyan);"></div>
        </div>
      </div>

      <!-- NTC Temp Bar (0 - 120 °C) -->
      <div class="racing-bar-wrapper">
        <div class="racing-bar-header">
          <span style="color: var(--text-muted);">SUHU ENGINE (NTC)</span>
          <span id="barTempVal" style="color: var(--neon-yellow); font-weight: bold;">0.0°C</span>
        </div>
        <div class="racing-bar-track">
          <div class="racing-bar-fill" id="barTempFill" style="background-color: var(--neon-yellow);"></div>
        </div>
      </div>

      <!-- Battery Voltage Bar (9.0V - 16.0V) -->
      <div class="racing-bar-wrapper">
        <div class="racing-bar-header">
          <span style="color: var(--text-muted);">BATT VOLTAGE</span>
          <span id="barBattVal" style="color: var(--neon-pink); font-weight: bold;">0.0V</span>
        </div>
        <div class="racing-bar-track">
          <div class="racing-bar-fill" id="barBattFill" style="background-color: var(--neon-pink);"></div>
        </div>
      </div>
    </div>

    <!-- Realtime Rolling Chart -->
    <div class="chart-card">
      <div class="chart-title">REAL-TIME ROLLING TELEMETRY CHART</div>
      <canvas id="realtimeChart" style="width:100%; height:160px;"></canvas>
      <div style="display:flex; justify-content:space-around; font-size:10px; font-family:'Orbitron'; margin-top:6px; color:var(--text-muted);">
        <span style="color:var(--neon-green);">■ RPM (Max 15k)</span>
        <span style="color:var(--neon-cyan);">■ ADVANCE (Max 45°)</span>
      </div>
    </div>
  </div>

  <!-- 2. TAB TUNING (MAP 3D & DWELL) -->
  <div id="tuningSection" style="display:none;">
    <div class="card">
      <div class="controls-grid">
        <div>
          <label>SLOT MAP CUSTOM</label>
          <select id="slotSelect" onchange="loadMap()">
            <option value="0">MAP CUSTOM 1</option>
            <option value="1">MAP CUSTOM 2</option>
            <option value="2">MAP CUSTOM 3</option>
            <option value="3">MAP CUSTOM 4</option>
            <option value="4">MAP CUSTOM 5</option>
          </select>
        </div>
        <div>
          <label>LIMIT RPM (HARD CUT)</label>
          <input type="number" id="rpmLimit" class="cfg" value="12500" step="100">
        </div>
      </div>
    </div>

    <div class="quick-adj-bar">
      <span style="font-size:10px; color:var(--text-muted); font-family:'Orbitron';">KOREKSI:</span>
      <button class="btn-adj" onclick="adjustSelectedCell(-10)">-1.0°</button>
      <button class="btn-adj" onclick="adjustSelectedCell(-5)">-0.5°</button>
      <button class="btn-adj" onclick="adjustSelectedCell(5)">+0.5°</button>
      <button class="btn-adj" onclick="adjustSelectedCell(10)">+1.0°</button>
    </div>

    <div class="tab-buttons" style="margin-bottom: 8px;">
      <button class="tab-btn active" id="subBtnDegree" onclick="switchSubTab('degree')">MAP DERAJAT</button>
      <button class="tab-btn" id="subBtnDwell" onclick="switchSubTab('dwell')">DWELL TIME</button>
    </div>

    <div class="chart-card">
      <div class="chart-title" id="chartTitle">GRAFIK KURVA PENGAPIAN (DEGREE VS RPM)</div>
      <canvas id="tuningChart" style="width:100%; height:160px;"></canvas>
    </div>

    <div id="tabDegree" class="table-responsive">
      <table>
        <thead><tr><th>RPM/TPS</th><th>0%</th><th>25%</th><th>50%</th><th>75%</th><th>100%</th></tr></thead>
        <tbody id="mapBody"></tbody>
      </table>
    </div>

    <div id="tabDwell" class="table-responsive" style="display:none;">
      <table>
        <thead><tr><th>STEP RPM</th><th>DWELL TIME (&mu;s)</th><th>KETERANGAN</th></tr></thead>
        <tbody id="dwellBody"></tbody>
      </table>
    </div>

    <button class="btn-save" onclick="saveAndSend()">SIMPAN & FLASH KE ECU</button>
  </div>

  <!-- 3. TAB DATA LOGGER -->
  <div id="loggerSection" style="display:none;">
    <div class="card" style="text-align: center;">
      <label style="color: var(--neon-cyan); margin-bottom: 6px;">RIWAYAT REKAMAN DATA MESIN</label>
      <div id="overflowBadge" style="display:none; color:#ff9100; font-size:11px; margin-bottom:10px; font-family:'Orbitron';">⚠️ RAM BUFFER MELUBER (LOG LAMA TERTIMPA)</div>
      <button class="btn-log" onclick="fetchLiveDataLog()" style="margin-bottom: 10px;">MUAT ULANG TABEL LOG</button>
      <a href="/downloadCSV" class="btn-log">DOWNLOAD FILE CSV</a>
      <button class="btn-clear" onclick="clearLogData()">BERSIHKAN LOG</button>
    </div>

    <div class="table-responsive">
      <table>
        <thead>
          <tr>
            <th>Waktu</th><th>RPM</th><th>TPS</th><th>Adv</th><th>Suhu</th><th>Aki</th>
          </tr>
        </thead>
        <tbody id="logTableBody">
          <tr><td colspan="6" style="color: var(--text-muted);">Klik "Muat Ulang Tabel Log" untuk menampilkan data.</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script>
const rpmLabels = ["1k","2k","3k","4k","5k","6k","7k","8k","9k","10k","11k","12k","13k","14k","15k"];
let lastFocusedCell = null;
let rtHistoryRpm = [];
let rtHistoryDeg = [];
const maxHistoryPoints = 60;

function switchMainTab(tab) {
  document.getElementById('liveSection').style.display = 'none';
  document.getElementById('tuningSection').style.display = 'none';
  document.getElementById('loggerSection').style.display = 'none';
  
  document.getElementById('btnTabLive').classList.remove('active');
  document.getElementById('btnTabDegree').classList.remove('active');
  document.getElementById('btnTabDwell').classList.remove('active');
  document.getElementById('btnTabLogger').classList.remove('active');

  if (tab === 'live') {
    document.getElementById('liveSection').style.display = 'block';
    document.getElementById('btnTabLive').classList.add('active');
    drawRealtimeChart();
  } else if (tab === 'degree') {
    document.getElementById('tuningSection').style.display = 'block';
    document.getElementById('btnTabDegree').classList.add('active');
    switchSubTab('degree');
  } else if (tab === 'dwell') {
    document.getElementById('tuningSection').style.display = 'block';
    document.getElementById('btnTabDwell').classList.add('active');
    switchSubTab('dwell');
  } else if (tab === 'logger') {
    document.getElementById('loggerSection').style.display = 'block';
    document.getElementById('btnTabLogger').classList.add('active');
    fetchLiveDataLog(); 
  }
}

function switchSubTab(sub) {
  if (sub === 'degree') {
    document.getElementById('tabDegree').style.display = 'block';
    document.getElementById('tabDwell').style.display = 'none';
    document.getElementById('subBtnDegree').classList.add('active');
    document.getElementById('subBtnDwell').classList.remove('active');
    document.getElementById('chartTitle').innerText = "GRAFIK KURVA PENGAPIAN (DEGREE VS RPM)";
  } else {
    document.getElementById('tabDegree').style.display = 'none';
    document.getElementById('tabDwell').style.display = 'block';
    document.getElementById('subBtnDegree').classList.remove('active');
    document.getElementById('subBtnDwell').classList.add('active');
    document.getElementById('chartTitle').innerText = "GRAFIK KURVA DWELL TIME (DWELL VS RPM)";
  }
  drawChart();
}

function pollLiveStatus() {
  fetch('/getLiveStatus').then(r => r.json()).then(data => {
    let statusEl = document.getElementById('ecuStatus');
    let rpmTopEl = document.getElementById('liveRpmTop');
    let faultEl = document.getElementById('faultBanner');
    
    if (data.ecuOnline) {
      statusEl.innerText = "ONLINE (CONNECTED)";
      statusEl.className = "ecu-online";
    } else {
      statusEl.innerText = "OFFLINE / DISCONNECTED";
      statusEl.className = "ecu-offline";
    }
    
    let rpm = data.rpm;
    let deg = data.deg / 10.0;
    let tps = data.tps;
    let temp = data.temp / 10.0;
    let batt = data.batt / 10.0;

    rpmTopEl.innerText = rpm;

    // Update Circular RPM Gauge
    document.getElementById('gaugeRpmVal').innerText = rpm;
    let rpmPercent = Math.min(Math.max((rpm / 15000) * 360, 0), 360);
    document.getElementById('rpmCircularGauge').style.background = `conic-gradient(var(--neon-green) ${rpmPercent}deg, #161622 ${rpmPercent}deg)`;

    // Update Racing Bars
    document.getElementById('barDegVal').innerText = deg.toFixed(1) + '°';
    document.getElementById('barDegFill').style.width = Math.min(Math.max((deg / 45.0) * 100, 0), 100) + '%';

    document.getElementById('barTpsVal').innerText = tps + '%';
    document.getElementById('barTpsFill').style.width = Math.min(Math.max(tps, 0), 100) + '%';

    document.getElementById('barTempVal').innerText = temp.toFixed(1) + '°C';
    document.getElementById('barTempFill').style.width = Math.min(Math.max((temp / 120.0) * 100, 0), 100) + '%';

    document.getElementById('barBattVal').innerText = batt.toFixed(1) + 'V';
    let battPercent = ((batt - 9.0) / (16.0 - 9.0)) * 100;
    document.getElementById('barBattFill').style.width = Math.min(Math.max(battPercent, 0), 100) + '%';

    rtHistoryRpm.push(rpm);
    rtHistoryDeg.push(deg);
    if (rtHistoryRpm.length > maxHistoryPoints) {
      rtHistoryRpm.shift();
      rtHistoryDeg.shift();
    }
    
    if (document.getElementById('liveSection').style.display !== 'none') {
      drawRealtimeChart();
    }

    if (data.faultCode && data.faultCode !== 0) {
      let msgs = [];
      if (data.faultCode & 1) msgs.push("SENSOR TPS ERROR");
      if (data.faultCode & 2) msgs.push("SENSOR SUHU NTC ERROR");
      if (data.faultCode & 4) msgs.push("AKI LOW (<10V)");
      if (data.faultCode & 8) msgs.push("OVERCHARGE / KIPROK (>16V)");
      faultEl.innerText = "⚠️ FAULT DETECTED: " + msgs.join(" | ");
      faultEl.style.display = "block";
    } else {
      faultEl.style.display = "none";
    }
  }).catch(e => {});
}
setInterval(pollLiveStatus, 300);

function drawRealtimeChart() {
  const canvas = document.getElementById('realtimeChart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  const w = canvas.width; const h = canvas.height;
  
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.04)'; ctx.lineWidth = 1;
  
  for(let y=0; y<h; y+=h/4) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  if (rtHistoryRpm.length < 2) return;

  let stepX = w / (maxHistoryPoints - 1);

  ctx.strokeStyle = 'var(--neon-green)'; ctx.lineWidth = 2; ctx.beginPath();
  for(let i = 0; i < rtHistoryRpm.length; i++) {
    let x = i * stepX;
    let y = h - ((rtHistoryRpm[i] / 15000) * h);
    if (y < 0) y = 0; if (y > h) y = h;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.strokeStyle = 'var(--neon-cyan)'; ctx.lineWidth = 1.5; ctx.beginPath();
  for(let i = 0; i < rtHistoryDeg.length; i++) {
    let x = i * stepX;
    let y = h - ((rtHistoryDeg[i] / 45.0) * h);
    if (y < 0) y = 0; if (y > h) y = h;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function fetchLiveDataLog() {
  fetch('/getLogData').then(r => r.json()).then(data => {
    let tbody = document.getElementById('logTableBody');
    tbody.innerHTML = '';
    
    if (data.overflow) {
      document.getElementById('overflowBadge').style.display = 'block';
    } else {
      document.getElementById('overflowBadge').style.display = 'none';
    }

    if (!data.logs || data.logs.length === 0) {
      tbody.innerHTML = '<tr><td colspan="6" style="color: var(--text-muted);">Belum ada data log.</td></tr>';
      return;
    }
    let revLogs = data.logs.reverse();
    revLogs.forEach(entry => {
      let tr = document.createElement('tr');
      let isTempAlert = (entry.temp / 10.0) >= 90.0;
      let isBattAlert = (entry.batt / 10.0) <= 11.5;
      
      if (isTempAlert || isBattAlert) tr.classList.add('tr-alert');
      
      tr.innerHTML = `
        <td style="color: var(--text-muted);">${(entry.t / 1000).toFixed(1)}s</td>
        <td style="color: var(--neon-green); font-weight: bold;">${entry.r}</td>
        <td>${entry.p}%</td>
        <td style="color: var(--neon-cyan);">${(entry.a / 10).toFixed(1)}&deg;</td>
        <td style="${isTempAlert ? 'color:var(--neon-pink); font-weight:900;' : ''}">${(entry.temp / 10).toFixed(1)}&deg;C</td>
        <td style="${isBattAlert ? 'color:var(--neon-pink); font-weight:900;' : ''}">${(entry.batt / 10).toFixed(1)}V</td>
      `;
      tbody.appendChild(tr);
    });
  }).catch(err => alert('Gagal mengambil data log.'));
}

function clearLogData() {
  if(confirm('Hapus seluruh riwayat log di memori?')) {
    fetch('/clearLog').then(r => r.text()).then(msg => {
      fetchLiveDataLog();
      alert('Log berhasil dibersihkan.');
    });
  }
}

function adjustSelectedCell(delta) {
  if (lastFocusedCell) {
    let currentVal = parseInt(lastFocusedCell.value) || 0;
    let newVal = currentVal + delta;
    if (newVal < 0) newVal = 0;
    if (newVal > 450) newVal = 450;
    lastFocusedCell.value = newVal;
    drawChart();
    checkPowerDropPatterns();
  } else {
    alert('Pilih/Klik salah satu kotak sel pada tabel Map 3D terlebih dahulu!');
  }
}

function checkPowerDropPatterns() {
  for(let r = 0; r < 15; r++) {
    for(let t = 0; t < 5; t++) {
      let cell = document.getElementById(`c_${r}_${t}`);
      if (cell) {
        let val = parseInt(cell.value) || 0;
        if (r >= 7 && val < 150) cell.classList.add('cell-drop');
        else cell.classList.remove('cell-drop');
      }
    }
  }
}

function drawChart() {
  const canvas = document.getElementById('tuningChart');
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  const w = canvas.width; const h = canvas.height;
  const padL = 30, padR = 15, padT = 15, padB = 25;
  const plotW = w - padL - padR; const plotH = h - padT - padB;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.lineWidth = 1; ctx.fillStyle = 'var(--text-muted)'; ctx.font = '9px Orbitron, sans-serif';
  
  let currentTabIsDegree = document.getElementById('tabDegree').style.display !== 'none';
  if (currentTabIsDegree) {
    let minY = 0, maxY = 450; 
    for(let v = 0; v <= 400; v += 100) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v/10) + '°', 5, y + 3);
    }
    for(let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
      ctx.fillText(rpmLabels[r], x - 8, h - 5);
    }
    const colors = ['var(--neon-cyan)', 'var(--neon-green)', 'var(--neon-yellow)', '#ff9100', 'var(--neon-pink)'];
    for(let t = 0; t < 5; t++) {
      ctx.strokeStyle = colors[t]; ctx.lineWidth = 2; ctx.beginPath();
      for(let r = 0; r < 15; r++) {
        let el = document.getElementById(`c_${r}_${t}`);
        let val = el ? (parseInt(el.value) || 0) : 100;
        let x = padL + (r / 14) * plotW;
        let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
        if(r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  } else {
    let minY = 1000, maxY = 5000;
    for(let v = 1000; v <= 5000; v += 1000) {
      let y = padT + plotH - ((v - minY) / (maxY - minY)) * plotH;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText((v/1000) + 'k', 5, y + 3);
    }
    for(let r = 0; r < 15; r += 2) {
      let x = padL + (r / 14) * plotW;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
      ctx.fillText(rpmLabels[r], x - 8, h - 5);
    }
    ctx.strokeStyle = 'var(--neon-cyan)'; ctx.lineWidth = 2.5; ctx.beginPath();
    for(let r = 0; r < 15; r++) {
      let el = document.getElementById(`d_${r}`);
      let val = el ? (parseInt(el.value) || 3200) : 3200;
      let x = padL + (r / 14) * plotW;
      let y = padT + plotH - ((val - minY) / (maxY - minY)) * plotH;
      if(r === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function loadMap() {
  let slot = document.getElementById('slotSelect').value;
  fetch('/getMap?slot=' + slot).then(r => r.json()).then(data => {
    document.getElementById('rpmLimit').value = data.rpmLimit;
    let tbodyMap = document.getElementById('mapBody'); tbodyMap.innerHTML = '';
    for(let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td');
      tdRpm.style.color = 'var(--neon-cyan)'; tdRpm.innerText = rpmLabels[r]; tr.appendChild(tdRpm);
      for(let t = 0; t < 5; t++) {
        let td = document.createElement('td');
        let val = (data.map && data.map[r]) ? data.map[r][t] : 100;
        td.innerHTML = `<input type="number" class="cell" id="c_${r}_${t}" value="${val}" onfocus="lastFocusedCell=this" oninput="drawChart(); checkPowerDropPatterns();">`;
        tr.appendChild(td);
      }
      tbodyMap.appendChild(tr);
    }
    let tbodyDwell = document.getElementById('dwellBody'); tbodyDwell.innerHTML = '';
    for(let r = 0; r < 15; r++) {
      let tr = document.createElement('tr');
      let tdRpm = document.createElement('td'); tdRpm.style.color = 'var(--neon-cyan)'; tdRpm.innerText = rpmLabels[r] + ' RPM';
      let tdDwell = document.createElement('td');
      let dVal = (data.dwell && data.dwell[r]) ? data.dwell[r] : 3200;
      tdDwell.innerHTML = `<input type="number" class="cell dwell-cell" id="d_${r}" value="${dVal}" step="50" oninput="drawChart()">`;
      let tdDesc = document.createElement('td'); tdDesc.style.color = 'var(--text-muted)'; tdDesc.style.fontSize = '11px';
      tdDesc.innerText = r < 4 ? 'Low RPM' : (r < 10 ? 'Mid RPM' : 'High RPM');
      tr.appendChild(tdRpm); tr.appendChild(tdDwell); tr.appendChild(tdDesc); tbodyDwell.appendChild(tr);
    }
    drawChart();
    checkPowerDropPatterns();
  });
}

function saveAndSend() {
  let slot = parseInt(document.getElementById('slotSelect').value);
  let payload = { slot: slot, rpmLimit: parseInt(document.getElementById('rpmLimit').value) || 12500, map: [], dwell: [] };
  for(let r = 0; r < 15; r++) {
    let row = [];
    for(let t = 0; t < 5; t++) { row.push(parseInt(document.getElementById(`c_${r}_${t}`).value) || 100); }
    payload.map.push(row);
  }
  for(let r = 0; r < 15; r++) { payload.dwell.push(parseInt(document.getElementById(`d_${r}`).value) || 3200); }
  
  fetch('/saveTuning', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload) })
  .then(r => r.text()).then(msg => alert('Map Custom ' + (slot+1) + ' Berhasil Disimpan & Dikirim ke ECU!'));
}

window.onload = function() {
  fetch('/getLiveStatus').then(r => r.json()).then(d => {
    document.getElementById('slotSelect').value = d.activeSlot;
    loadMap();
  }).catch(e => loadMap());
};
window.onresize = function() {
  drawRealtimeChart();
  drawChart();
};
</script>
</body></html>
)rawliteral";

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
    lastActivityTime = millis();
    request->send_P(200, "text/html", htmlUI); 
  });
  
  server.on("/getLiveStatus", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    JsonDocument doc;
    
    portENTER_CRITICAL(&dataMux);
    doc["rpm"] = currentTelemetry.rpm;
    doc["tps"] = currentTelemetry.tps;
    doc["deg"] = currentTelemetry.degree10;
    doc["temp"] = currentTelemetry.temp10;
    doc["batt"] = currentTelemetry.battVolt10;
    doc["faultCode"] = currentTelemetry.faultCode;
    doc["ackStatus"] = currentTelemetry.ackStatus;
    portEXIT_CRITICAL(&dataMux);

    doc["ecuOnline"] = isEcuConnected;
    doc["activeSlot"] = currentActiveSlot;

    String resp;
    serializeJson(doc, resp);
    request->send(200, "application/json", resp);
  });

  server.on("/getMap", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    uint8_t slot = request->hasParam("slot") ? request->getParam("slot")->value().toInt() : 0;
    if (slot > 4) slot = 0;
    JsonDocument doc;
    doc["rpmLimit"] = customMapSlots[slot].rpmLimit;
    JsonArray arrMap = doc["map"].to<JsonArray>();
    for (int r = 0; r < NUM_RPM_POINTS; r++) {
      JsonArray row = arrMap.add<JsonArray>();
      for (int t = 0; t < NUM_TPS_POINTS; t++) row.add(customMapSlots[slot].mapData[r][t]);
    }
    JsonArray arrDwell = doc["dwell"].to<JsonArray>();
    for (int r = 0; r < NUM_RPM_POINTS; r++) arrDwell.add(customMapSlots[slot].dwellData[r]);
    String response; serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/getLogData", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument doc;
    doc["overflow"] = bufferOverflowed;
    
    JsonArray arr = doc["logs"].to<JsonArray>();
    portENTER_CRITICAL(&logMux);
    uint16_t startIdx = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (uint16_t i = 0; i < logCount; i++) {
      uint16_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
      JsonObject entry = arr.add<JsonObject>();
      entry["t"] = logBuffer[idx].timestampMs;
      entry["r"] = logBuffer[idx].rpm;
      entry["p"] = logBuffer[idx].tps;
      entry["a"] = logBuffer[idx].degree10;
      entry["temp"] = logBuffer[idx].temp10;
      entry["batt"] = logBuffer[idx].battVolt10;
    }
    portEXIT_CRITICAL(&logMux);
    serializeJson(doc, *response);
    request->send(response);
  });

  server.on("/clearLog", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    portENTER_CRITICAL(&logMux);
    logHead = 0; logCount = 0; bufferOverflowed = false;
    portEXIT_CRITICAL(&logMux);
    request->send(200, "text/plain", "Log Cleared");
  });

  server.on("/downloadCSV", HTTP_GET, [](AsyncWebServerRequest *request){
    lastActivityTime = millis();
    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    response->addHeader("Content-Disposition", "attachment; filename=satria_ecu_log.csv");
    response->print("Timestamp_ms,RPM,TPS_Pct,Adv_Deg,Temp_C,Batt_V\n");
    portENTER_CRITICAL(&logMux);
    uint16_t startIdx = (logCount < MAX_LOG_ENTRIES) ? 0 : logHead;
    for (uint16_t i = 0; i < logCount; i++) {
      uint16_t idx = (startIdx + i) % MAX_LOG_ENTRIES;
      response->printf("%lu,%u,%u,%.1f,%.1f,%.1f\n",
                       logBuffer[idx].timestampMs, logBuffer[idx].rpm, logBuffer[idx].tps,
                       (float)logBuffer[idx].degree10 / 10.0f,
                       (float)logBuffer[idx].temp10 / 10.0f,
                       (float)logBuffer[idx].battVolt10 / 10.0f);
    }
    portEXIT_CRITICAL(&logMux);
    request->send(response);
  });

  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/saveTuning", [](AsyncWebServerRequest *request, JsonVariant &json) {
    lastActivityTime = millis();
    JsonObject obj = json.as<JsonObject>();
    uint8_t slot = obj["slot"].as<uint8_t>();
    if (slot > 4) slot = 0;
    
    currentActiveSlot = slot;
    customMapSlots[slot].rpmLimit = obj["rpmLimit"].as<uint16_t>();
    
    JsonArray arrMap = obj["map"].as<JsonArray>(); int r = 0;
    for(JsonVariant row : arrMap) {
      if(r >= NUM_RPM_POINTS) break; int t = 0;
      for(JsonVariant val : row.as<JsonArray>()) { 
        if(t >= NUM_TPS_POINTS) break; 
        customMapSlots[slot].mapData[r][t] = val.as<int16_t>(); t++; 
      } r++;
    }
    JsonArray arrDwell = obj["dwell"].as<JsonArray>(); int d = 0;
    for(JsonVariant val : arrDwell) {
      if(d >= NUM_RPM_POINTS) break;
      customMapSlots[slot].dwellData[d] = val.as<uint16_t>();
      d++;
    }
    saveMapToNVS();
    sendCommandToECU(0x03, slot); 
    request->send(200, "text/plain", "Saved & Sent");
    lastActivityTime = millis();
  });
  
  server.addHandler(handler);
  server.begin();
}

void setup() {
  setCpuFrequencyMhz(80); 
  Serial1.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ECU_SATRIA_FU", "kudus1234", 1); 
  esp_wifi_set_max_tx_power(50);
  
  esp_wifi_set_mac(WIFI_IF_STA, transmitterMac);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnEspNowSent);
    esp_now_register_recv_cb(OnEspNowRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, displayMacAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  loadMapFromNVS();
  setupWebServer();
  
  lastActivityTime = millis();
}

void loop() {
  parseUART();
  
  if (isEcuConnected && !initialSyncDone) {
    if (millis() - lastSyncRetryTime > 500) {
      sendCommandToECU(0x02, currentActiveSlot);
      lastSyncRetryTime = millis();
      syncRetryCount++;
      
      if (syncRetryCount >= 10) {
        initialSyncDone = true;
      }
    }
  }

  if (millis() - lastUartRxTime > UART_TIMEOUT_MS) {
    isEcuConnected = false;
    initialSyncDone = false;
    syncRetryCount = 0;
    portENTER_CRITICAL(&dataMux);
    currentTelemetry.rpm = 0;
    portEXIT_CRITICAL(&dataMux);
  }

  if (isWifiActive && (millis() - lastActivityTime > WIFI_TIMEOUT_MS)) {
    server.end();                 
    WiFi.softAPdisconnect(true);  
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    isWifiActive = false;         
  }
  
  delay(1);
}
