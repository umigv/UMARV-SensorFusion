/*
 * ESP32 SPI Host + Optional Live VN-300 Bridge for UMARV Sensor-Fusion Chip on Digilent Genesys 2.
 *
 * Wiring (all 3.3 V directly — NEVER connect Pmod Pins 6/12 3.3V!):
 *   Pmod JC (Host SPI):
 *     ESP32 GPIO 18 (SCK)  -> Pmod JC Pin 1 (jc_sck,  AC26)
 *     ESP32 GPIO  5 (CS_N) -> Pmod JC Pin 2 (jc_cs_n, AJ27)
 *     ESP32 GPIO 23 (MOSI) -> Pmod JC Pin 3 (jc_mosi, AH30)
 *     ESP32 GPIO 19 (MISO) -> Pmod JC Pin 4 (jc_miso, AK29)
 *     ESP32 GND            -> Pmod JC Pin 5 or 11 (GND)
 *     ESP32 GPIO  4 (IRQ)  -> Pmod JC Pin 7 (jc_meas_ready, AD26) [optional]
 *
 *   Pmod JD (Optional live PC-USB VN-300 -> ESP32 -> FPGA bridge):
 *     ESP32 GPIO 16 (UART2 TX) -> Pmod JD Pin 1 (jd_vn_rx,   V27) @ 921600 baud
 *     ESP32 GPIO 17 (SYNC IN)  <- Pmod JD Pin 2 (jd_vn_sync, Y30) 10 us pulse @ 400 Hz
 *
 * Serial console (115200 baud) commands:
 *   d              Dump ID, version, control, event counters, and sensor measurements
 *   c              Clear event counters (write reg 0x3D)
 *   p <dec_port>   Set lidar UDP port (0 = accept any UDP port; default 2368)
 *   u 0|1          Use UKF filter state + motion correction (CTRL=0xF3) vs host/off (0x43)
 *   b 0|1          Lidar bypass off/on
 *   r <hex_addr>   Read 1 register (e.g. "r 00")
 *   w <hex_addr> <hex_val>  Write 1 register (e.g. "w 02 73")
 *   s <omega> <v>  Inject host pose state (turn rate rad/s, speed m/s) & strobe 0x3C
 *   v <112-hex>    Load 56-byte VN-300 Binary Output 1 frame & enable sync-triggered TX on GPIO 16
 *   v 0            Disable VN-300 bridge TX on GPIO 16
 *   a 0|1          Enable/disable periodic 2s auto-dump (default 0)
 *   R <hex_addr> <dec_len>   Burst read up to 64 regs -> "@R <AA> <HEX>"
 *   W <hex_addr> <hex_bytes> Burst write up to 64 regs -> "@W <AA> <READBACK_HEX>"
 *   S              Bridge + 400 Hz dead-wheel status -> "@S <sync> <vn_tx> <br_en> <dw_abs> <dw_beats> <dw_ok>"
 *   Z              Zero 400 Hz dead-wheel absolute count accumulator -> "@Z OK"
 */
#include <SPI.h>

static const int PIN_SCK     = 18;
static const int PIN_MISO    = 19;
static const int PIN_MOSI    = 23;
static const int PIN_CS      = 5;
static const int PIN_READY   = 4;
static const int PIN_VN_TX   = 16; // -> Pmod JD Pin 1 (jd_vn_rx)
static const int PIN_VN_SYNC = 17; // <- Pmod JD Pin 2 (jd_vn_sync)

static const SPISettings CHIP_SPI(2000000, MSBFIRST, SPI_MODE0); // 2 MHz (max 5 MHz)

static volatile bool     g_vnSyncFlag  = false;
static volatile uint32_t g_vnSyncCount = 0;
static volatile uint32_t g_lastSyncUs  = 0;
static volatile bool     g_dwSampleDue = false;
static uint8_t           g_vnFrame[56];
static volatile bool     g_vnBridgeEn  = false;
static volatile uint32_t g_vnTxCount   = 0;
static volatile uint32_t g_vnLateDrop  = 0;
static bool              g_autoDump    = false;
static portMUX_TYPE      g_vnMux       = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t      g_vnTask      = nullptr;

// Exact 400 Hz beat-synchronized dead-wheel position accumulator
static int64_t           g_dwAbsCounts = 0;
static uint32_t          g_dwGoodBeats = 0;
static bool              g_dwLastValid = false;
static int32_t           g_dwLastDelta = 0;

void IRAM_ATTR onVnSyncRise() {
  g_vnSyncFlag = true;
  g_vnSyncCount++;
  g_lastSyncUs = micros();
  g_dwSampleDue = true;
  if (g_vnTask != nullptr) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_vnTask, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
  }
}

// Dedicated high-priority Core-0 task so 400 Hz VN-300 TX fires within <15 us of
// jd_vn_sync even while Core-1 is busy with USB Serial / SPI commands. Never sends
// a late packet (>250 us after sync) so the UKF always finishes before the next beat.
static void vnSyncTask(void * /*arg*/) {
  uint8_t localFrame[56];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!g_vnBridgeEn) continue;
    uint32_t age = (uint32_t)(micros() - g_lastSyncUs);
    if (age > 250) {
      g_vnLateDrop++;
      continue;
    }
    portENTER_CRITICAL(&g_vnMux);
    memcpy(localFrame, g_vnFrame, 56);
    portEXIT_CRITICAL(&g_vnMux);
    Serial2.write(localFrame, 56); // fits in 128B HW UART2 TX FIFO (<5 us)
    g_vnTxCount++;
  }
}

void chipRead(uint8_t addr, uint8_t *buf, size_t n) {
  SPI.beginTransaction(CHIP_SPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr & 0x7F);
  for (size_t i = 0; i < n; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void chipWrite(uint8_t addr, const uint8_t *buf, size_t n) {
  SPI.beginTransaction(CHIP_SPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0x80 | (addr & 0x7F));
  for (size_t i = 0; i < n; i++) {
    SPI.transfer(buf[i]);
  }
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

uint8_t chipRead8(uint8_t addr) {
  uint8_t v = 0;
  chipRead(addr, &v, 1);
  return v;
}

void chipWrite8(uint8_t addr, uint8_t val) {
  chipWrite(addr, &val, 1);
}

void dumpAll() {
  uint8_t hdr[4];
  chipRead(0x00, hdr, 4);
  Serial.printf("ID=0x%02X (expect 0xA7)  VER=0x%02X  CTRL=0x%02X  MODEL=%u\n",
                hdr[0], hdr[1], hdr[2], hdr[3] & 0x03);

  uint8_t cnt[7];
  chipRead(0x40, cnt, 7);
  Serial.printf("Counters: lidar=%u other=%u rx_crc_bad=%u sent=%u buf_ovf=%u mot_ovr=%u tx_und=%u\n",
                cnt[0], cnt[1], cnt[2], cnt[3], cnt[4], cnt[5], cnt[6]);

  uint8_t meas[36];
  chipRead(0x48, meas, 36); // 0x48..0x6B
  uint32_t beat_ts;
  memcpy(&beat_ts, &meas[0], 4);
  uint8_t vbits = meas[4];
  float yaw, rate;
  double lat, lon;
  int32_t dw_delta;
  memcpy(&yaw,      &meas[0x50 - 0x48], 4);
  memcpy(&rate,     &meas[0x54 - 0x48], 4);
  memcpy(&lat,      &meas[0x58 - 0x48], 8);
  memcpy(&lon,      &meas[0x60 - 0x48], 8);
  memcpy(&dw_delta, &meas[0x68 - 0x48], 4);

  Serial.printf("Beat=%lu us  vn_valid=%u  dw_valid=%u  (sync_irq=%lu vn_tx=%lu)\n",
                (unsigned long)beat_ts, vbits & 1, (vbits >> 1) & 1,
                (unsigned long)g_vnSyncCount, (unsigned long)g_vnTxCount);
  Serial.printf("  VN-300: yaw=%.3f deg  rate=%.4f rad/s  lat=%.7f  lon=%.7f\n",
                yaw, rate, lat, lon);
  Serial.printf("  AMT22 : dw_delta=%ld counts\n", (long)dw_delta);
}

void writeHostState(float omega, float v) {
  uint8_t buf[24] = {0};
  uint32_t ts = (uint32_t)micros();
  memcpy(&buf[0],  &ts,    4);
  memcpy(&buf[16], &omega, 4);
  memcpy(&buf[20], &v,     4);
  chipWrite(0x24, buf, 24);
  chipWrite8(0x3C, 1); // strobe state_valid
  Serial.printf("Injected host state: ts=%lu omega=%.3f rad/s v=%.3f m/s\n",
                (unsigned long)ts, omega, v);
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void handleLine(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0') return;
  char cmd = *s;
  if (cmd == 'd') {
    dumpAll();
  } else if (cmd == 'c') {
    chipWrite8(0x3D, 1);
    Serial.println("Counters cleared.");
  } else if (cmd == 'p') {
    unsigned int port = 0;
    if (sscanf(s + 1, "%u", &port) == 1) {
      uint8_t lo = port & 0xFF, hi = (port >> 8) & 0xFF;
      chipWrite8(0x08, lo);
      chipWrite8(0x09, hi);
      uint16_t rb = chipRead8(0x08) | ((uint16_t)chipRead8(0x09) << 8);
      Serial.printf("Lidar UDP port set to %u (readback=%u)\n", port, rb);
    }
  } else if (cmd == 'u') {
    int en = atoi(s + 1);
    uint8_t ctrl = en ? 0xF3 : 0x43;
    chipWrite8(0x02, ctrl);
    Serial.printf("UKF state + motion correction %s (CTRL=0x%02X, readback=0x%02X)\n",
                  en ? "ON" : "OFF", ctrl, chipRead8(0x02));
  } else if (cmd == 'b') {
    int en = atoi(s + 1);
    uint8_t ctrl = chipRead8(0x02);
    ctrl = en ? (ctrl | 0x04) : (ctrl & ~0x04);
    chipWrite8(0x02, ctrl);
    Serial.printf("Bypass %s (CTRL=0x%02X)\n", en ? "ON" : "OFF", ctrl);
  } else if (cmd == 'a') {
    g_autoDump = (atoi(s + 1) != 0);
    Serial.printf("Auto-dump %s\n", g_autoDump ? "ON" : "OFF");
  } else if (cmd == 'r') {
    unsigned int addr = 0;
    if (sscanf(s + 1, "%x", &addr) == 1) {
      Serial.printf("Reg 0x%02X = 0x%02X\n", addr, chipRead8((uint8_t)addr));
    }
  } else if (cmd == 'w') {
    unsigned int addr = 0, val = 0;
    if (sscanf(s + 1, "%x %x", &addr, &val) == 2) {
      chipWrite8((uint8_t)addr, (uint8_t)val);
      Serial.printf("Wrote Reg 0x%02X = 0x%02X (readback 0x%02X)\n",
                    addr, val, chipRead8((uint8_t)addr));
    }
  } else if (cmd == 's') {
    float omega = 0, v = 0;
    if (sscanf(s + 1, "%f %f", &omega, &v) == 2) {
      writeHostState(omega, v);
    }
  } else if (cmd == 'v') {
    const char *p = s + 1;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '0' && (*(p + 1) == '\0' || *(p + 1) == ' ')) {
      g_vnBridgeEn = false;
      Serial.println("VN-300 bridge OFF");
      return;
    }
    uint8_t tmp[56];
    for (int i = 0; i < 56; i++) {
      int hi = hexVal(p[2 * i]);
      int lo = hexVal(p[2 * i + 1]);
      if (hi < 0 || lo < 0) {
        Serial.println("ERR: need 112 hex chars (56 bytes)");
        return;
      }
      tmp[i] = (uint8_t)((hi << 4) | lo);
    }
    portENTER_CRITICAL(&g_vnMux);
    memcpy(g_vnFrame, tmp, 56);
    portEXIT_CRITICAL(&g_vnMux);
    g_vnBridgeEn = true;
  } else if (cmd == 'R') {
    unsigned int addr = 0, len = 0;
    if (sscanf(s + 1, "%x %u", &addr, &len) == 2 && len > 0 && len <= 64) {
      uint8_t buf[64];
      chipRead((uint8_t)addr, buf, len);
      Serial.printf("@R %02X ", addr & 0x7F);
      for (unsigned int i = 0; i < len; i++) Serial.printf("%02X", buf[i]);
      Serial.println();
    } else {
      Serial.println("@ERR R");
    }
  } else if (cmd == 'W') {
    const char *p = s + 1;
    while (*p == ' ' || *p == '\t') p++;
    int ah = hexVal(p[0]), al = hexVal(p[1]);
    if (ah < 0 || al < 0) { Serial.println("@ERR W"); return; }
    uint8_t addr = (uint8_t)((ah << 4) | al);
    p += 2;
    while (*p == ' ' || *p == '\t') p++;
    uint8_t buf[64];
    size_t len = 0;
    while (p[0] && p[1] && len < 64) {
      int hi = hexVal(p[0]), lo = hexVal(p[1]);
      if (hi < 0 || lo < 0) break;
      buf[len++] = (uint8_t)((hi << 4) | lo);
      p += 2;
    }
    if (len == 0) { Serial.println("@ERR W"); return; }
    chipWrite(addr, buf, len);
    uint8_t rb[64];
    chipRead(addr, rb, len);
    Serial.printf("@W %02X ", addr & 0x7F);
    for (size_t i = 0; i < len; i++) Serial.printf("%02X", rb[i]);
    Serial.println();
  } else if (cmd == 'S') {
    Serial.printf("@S %lu %lu %u %lld %lu %u\n",
                  (unsigned long)g_vnSyncCount,
                  (unsigned long)g_vnTxCount,
                  g_vnBridgeEn ? 1 : 0,
                  (long long)g_dwAbsCounts,
                  (unsigned long)g_dwGoodBeats,
                  g_dwLastValid ? 1 : 0);
  } else if (cmd == 'Z') {
    g_dwAbsCounts = 0;
    g_dwGoodBeats = 0;
    Serial.println("@Z OK");
  }
}

static void sampleDeadWheelBeat() {
  // Runs ~2100 us after jd_vn_sync (after both VN-300 ~656 us and idle 2001 us deadline)
  // while dw_valid and dw_delta are frozen and steady for the beat (~28 us SPI burst).
  uint8_t vbits = chipRead8(0x4C);
  if (vbits & 0x02) {
    uint8_t raw[4];
    chipRead(0x68, raw, 4);
    int32_t d = 0;
    memcpy(&d, raw, 4);
    // Unwrap single-turn 14-bit AMT22 boundary crossings (16383 -> 0 gives -16383, etc.).
    // At 400 Hz, >8192 counts/beat would require >12,000 RPM, so this is safe for both
    // single-turn and multi-turn encoders.
    if (d > 8192 && d < 16384)       d -= 16384;
    else if (d < -8192 && d > -16384) d += 16384;
    g_dwAbsCounts += (int64_t)d;
    g_dwGoodBeats++;
    g_dwLastDelta = d;
    g_dwLastValid = true;
  } else {
    g_dwLastValid = false;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_READY, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // VN-300 bridge: 921600 baud TX on GPIO 16 -> JD1, sync input on GPIO 17 <- JD2
  Serial2.begin(921600, SERIAL_8N1, -1, PIN_VN_TX);
  xTaskCreatePinnedToCore(vnSyncTask, "vnSyncTask", 2048, nullptr, configMAX_PRIORITIES - 2, &g_vnTask, 0);
  pinMode(PIN_VN_SYNC, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_VN_SYNC), onVnSyncRise, RISING);

  delay(200);
  Serial.println("\nUMARV Sensor-Fusion Chip ESP32 SPI Host + VN-300 Bridge");
  dumpAll();
}

static char lineBuf[160];
static size_t lineLen = 0;
static unsigned long lastPrint = 0;

void loop() {
  // 1. Beat-synchronized dead-wheel accumulation (~2100 us after beat, after collect_end)
  // (VN-300 400 Hz TX is handled deterministically on Core 0 by vnSyncTask.)
  if (g_dwSampleDue) {
    uint32_t elapsed = (uint32_t)(micros() - g_lastSyncUs);
    if (elapsed >= 2100 && elapsed < 2450) {
      g_dwSampleDue = false;
      sampleDeadWheelBeat();
    } else if (elapsed >= 2450) {
      g_dwSampleDue = false; // missed window, skip cleanly
    }
  }

  // Non-blocking serial console parser
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  if (g_autoDump && (millis() - lastPrint >= 2000)) {
    lastPrint = millis();
    dumpAll();
  }
}
