#include <Arduino.h>
#include <math.h>

// Gunakan pin UART yang sama atau sesuaikan dengan jalur jumper Anda
// Jika TX Simulator ke RX Telemetri (Pin 7):
#define SIM_TX_PIN 6
#define SIM_RX_PIN 7

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5

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

struct __attribute__((packed)) CommandPacketToECU {
  uint16_t header;                                  // 0x55CC
  uint8_t  cmdType;
  uint8_t  slotOrMode;
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS];
  uint16_t dwellData[NUM_RPM_POINTS];
  uint16_t rpmLimit;
  uint16_t crc16;
};

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void listenCommandsFromTelemetry() {
  static uint8_t cmdBuf[sizeof(CommandPacketToECU)];
  static size_t idx = 0;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (idx == 0) {
      if (c == 0x55) cmdBuf[idx++] = c;
    } else if (idx == 1) {
      if (c == 0xCC) cmdBuf[idx++] = c;
      else idx = (c == 0x55) ? 1 : 0;
    } else {
      cmdBuf[idx++] = c;
      if (idx >= sizeof(CommandPacketToECU)) {
        CommandPacketToECU *pkt = (CommandPacketToECU*)cmdBuf;
        uint16_t calc = calculateCRC16(cmdBuf, sizeof(CommandPacketToECU) - sizeof(uint16_t));
        if (pkt->crc16 == calc) {
          // Ada perintah tuning/sync masuk dari C3 Telemetri
          // Anda bisa memantau lewat Serial USB jika diperlukan
        }
        idx = 0;
      }
    }
  }
}

void setup() {
  Serial1.begin(250000, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
}

void loop() {
  static unsigned long lastSend = 0;
  static float phase = 0.0f;

  listenCommandsFromTelemetry();

  if (millis() - lastSend >= 50) { // Kirim setiap 50ms (20 Hz)
    lastSend = millis();
    phase += 0.05f;
    if (phase > 6.283185f) phase -= 6.283185f;

    float rpmNorm = (sinf(phase) + 1.0f) * 0.5f;
    float tpsNorm = (sinf(phase - 0.6f) + 1.0f) * 0.5f;

    TelemetryData pkt = {};
    pkt.header     = 0xAA55;
    pkt.rpm        = (uint16_t)(1400.0f + rpmNorm * 12600.0f); // 1.400 - 14.000 RPM
    pkt.tps        = (uint8_t)(tpsNorm * 100.0f);              // 0 - 100%
    pkt.degree10   = (int16_t)(120 + rpmNorm * 230);           // 12.0° - 35.0°
    pkt.temp10     = (int16_t)(750 + sinf(phase * 0.2f) * 150);// ~60.0°C - 90.0°C
    pkt.battVolt10 = (uint16_t)(132 + cosf(phase * 0.4f) * 8);  // ~12.4V - 14.0V
    pkt.mode       = 1;
    pkt.fuel       = 80;
    pkt.state      = 1;
    pkt.ackStatus  = 1;
    pkt.faultCode  = 0;

    pkt.crc16 = calculateCRC16((uint8_t*)&pkt, sizeof(TelemetryData) - sizeof(uint16_t));
    Serial1.write((uint8_t*)&pkt, sizeof(TelemetryData));
  }
}
