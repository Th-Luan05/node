#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include "proto.h"
#include <nvs_flash.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>

// Phiên bản firmware node. PHẢI bằng số N trong tên file node_N.bin trên server; tăng lên mỗi lần phát hành.
#define FW_VERSION  1
// Chứng chỉ gốc (PEM) của server OTA, nên có để chống giả mạo server:
// #define OTA_ROOT_CA "-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----\n"

#define LIGHT_PIN       35      // Chân ADC nối điểm giữa LDR và R_LDR_FIXED
#define NTC_PIN         34      // Chân ADC nối điểm giữa NTC và R_FIXED
#define VCC_MV          3333.0f // mV (3.335V)
#define R_FIXED         2435.1f // Ohm
#define BETA            3950.0f

#define T_MIN          -40
#define T_MAX           200
#define TABLE_SIZE      (T_MAX - T_MIN + 1) // 241 giá trị

#define R_LDR_FIXED     10000.0f // Trở 10k nối GND cho quang trở
#define SAMPLES         32       // Số mẫu trung bình khử nhiễu ADC

// Bảng điện trở (đơn vị: Ohm) từ -40°C đến 200°C (bước 1°C) lưu trong Flash
const float PROGMEM R_TABLE[TABLE_SIZE] = {
  // -40°C đến 0°C
  277200, 263600, 250100, 236800, 224000, 211500, 199600, 188100, 177300, 167000,
  157200, 148100, 139400, 131300, 123700, 116600, 110000, 103700, 97900,  92500,
  87430,  82790,  78440,  74360,  70530,  66920,  63540,  60340,  57330,  54500,
  51820,  49280,  46890,  44620,  42480,  40450,  38530,  36700,  34970,  33330,
  31770,
  // 1°C đến 41°C
  30250,  28820,  27450,  26160,  24940,  23770,  22670,  21620,  20630,  19680,
  18780,  17930,  17120,  16350,  15620,  14930,  14260,  13630,  13040,  12470,
  11920,  11410,  10910,  10450,  10000,  9575,   9170,   8784,   8416,   8064,
  7730,   7410,   7106,   6815,   6538,   6273,   6020,   5778,   5548,   5327,
  5117,
  // 42°C đến 82°C
  4915,   4723,   4539,   4363,   4195,   4034,   3880,   3733,   3592,   3457,
  3328,   3204,   3086,   2972,   2863,   2759,   2659,   2564,   2472,   2384,
  2299,   2218,   2141,   2066,   1994,   1926,   1860,   1796,   1735,   1677,
  1621,   1567,   1515,   1465,   1417,   1371,   1326,   1284,   1243,   1203,
  1165,
  // 83°C đến 123°C
  1128,   1093,   1059,   1027,   995.5,  965.4,  936.3,  908.3,  881.2,  855.0,
  829.7,  805.2,  781.6,  758.7,  736.6,  715.2,  694.5,  674.4,  655.8,  637.6,
  619.9,  602.6,  585.8,  569.4,  553.5,  538.0,  522.9,  508.3,  494.1,  480.3,
  466.9,  453.9,  441.2,  429.0,  417.1,  405.5,  394.4,  383.5,  373.0,  362.8,
  353.0,
  // 124°C đến 164°C
  343.4,  334.1,  325.3,  316.7,  308.3,  300.2,  292.4,  284.8,  277.4,  270.2,
  263.3,  256.5,  250.0,  243.7,  237.5,  231.6,  225.8,  220.2,  214.8,  209.5,
  204.4,  199.4,  194.6,  190.0,  185.5,  181.1,  176.9,  172.8,  168.8,  165.0,
  161.2,  157.6,  154.1,  150.7,  147.4,  144.1,  141.0,  137.9,  135.0,  132.1,
  129.3,
  // 165°C đến 200°C
  126.5,  123.9,  121.3,  118.7,  116.3,  113.9,  111.5,  109.2,  107.0,  104.8,
  102.7,  100.6,  98.6,   96.6,   94.7,   92.8,   90.9,   89.1,   87.3,   85.6,
  83.9,   82.2,   80.6,   79.0,   77.4,   75.9,   74.3,   72.9,   71.4,   70.0,
  68.6,   67.2,   65.8,   64.5,   63.1,   61.9
};

float readAdcMilliVolts(int pin, int samples = 30) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(pin);
    delayMicroseconds(400);
  }
  return sum / samples;
}

float calculateRntc(float vAdc) {
  if (vAdc <= 1.0f) vAdc = 1.0f;
  if (vAdc >= VCC_MV) vAdc = VCC_MV - 1.0f;
  return R_FIXED * ((VCC_MV - vAdc) / vAdc);
}

float calculateTemperature(float rNtc) {
  float rMinTemp = pgm_read_float(&R_TABLE[0]);
  float rMaxTemp = pgm_read_float(&R_TABLE[TABLE_SIZE - 1]);

  if (rNtc >= rMinTemp) return (float)T_MIN;
  if (rNtc <= rMaxTemp) return (float)T_MAX;

  int low = 0;
  int high = TABLE_SIZE - 1;
  int index = 0;

  while (low <= high) {
    int mid = low + (high - low) / 2;
    float rMid = pgm_read_float(&R_TABLE[mid]);

    if (rMid >= rNtc) {
      index = mid;
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }

  int t1 = T_MIN + index;
  float r1 = pgm_read_float(&R_TABLE[index]);

  float t1_K = (float)t1 + 273.15f;
  float invT = (1.0f / t1_K) + (1.0f / BETA) * logf(rNtc / r1);
  float t_K = 1.0f / invT;

  return t_K - 273.15f;
}

// Tận dụng hàm tb() có sẵn, thay delay(200) bằng microsecond để vòng loop chạy nhanh và mượt
float tb(int val, int samples = SAMPLES) {
  uint32_t trb = 0;
  for (int i = 0; i < samples; i++) {
    trb += analogRead(val);
    delayMicroseconds(300);
  }
  return (float)trb / samples;
}

// Tính Lux từ giá trị ADC trả về bởi hàm tb()
float calculateLux(float adc) {
  if (adc < 30.0f) return 0.5f;       // Giới hạn dưới (< 1 Lux)
  if (adc >= 4080.0f) adc = 4080.0f; // Bão hòa dải đo

  // 1. Tính điện trở LDR từ cầu phân áp 10k
  float r_ldr = R_LDR_FIXED * ((4095.0f - adc) / adc);

  // 2. Logarit hóa điện trở
  float u = logf(r_ldr);

  // 3. Phương trình thực nghiệm bậc 2 đã khớp theo bảng dữ liệu
  float ln_lux = -0.0215f * u * u - 0.7745f * u + 14.246f;

  return expf(ln_lux);
}


// =====================================================================
//  PHẦN MỚI: relay + BLE provisioning + Wi-Fi/UDP + ESP-NOW chuyển tiếp
// =====================================================================
#define RELAY_PIN            27
#define RELAY_ACTIVE_LEVEL   HIGH     // module relay kích mức thấp thì đổi thành LOW

#define BEACON_MS            3000
#define TELEMETRY_MS         5000
#define PARENT_TIMEOUT_MS    10000
#define UPLINK_TIMEOUT_MS    20000
#define BLE_WINDOW_MS        120000   // sau khi bật nguồn, BLE mở 2 phút để master cấu hình lại

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t    myMac[6];
static prov_blob_t prov;
static bool       provValid = false;
static prov_blob_t pendingBlob;
static volatile bool provPending = false;
static bool       bleOn = false;

static WiFiUDP    udp;
static bool       udpStarted = false;
static IPAddress  masterIp;
static uint32_t   lastAckMs = 0;

static bool       relayOn = false;
static float      gTemp = 0, gLux = 0;
static uint16_t   seqNo = 0;

// ---------- thông tin thiết bị do người dùng đặt (lưu NVS) ----------
struct dev_cfg_t { uint8_t configured; uint8_t area; uint8_t dtype; char name[NAME_LEN]; };
static dev_cfg_t dev = {};          // configured = 0  ->  thiết bị mới

static void sendTelemetry();        // khai báo trước (định nghĩa bên dưới)

// ---------- bảo vệ quá nhiệt: chạy hoàn toàn tại node, KHÔNG phụ thuộc mạng ----------
#define SAFETY_MS          1000    // chu kỳ đo nhiệt độ riêng cho bảo vệ
#define OT_CONFIRM_COUNT   3       // số lần đo liên tiếp vượt ngưỡng mới coi là thật (lọc nhiễu)
#define OT_WARN_HYST       2.0f    // °C: nhiệt độ < (ngưỡng cảnh báo - giá trị này) thì tắt cảnh báo
#define OT_CLEAR_MARGIN    5.0f    // °C: chỉ cho mở khóa khi nhiệt độ < (ngưỡng ngắt - giá trị này)

struct safety_cfg_t { int16_t warn_c; int16_t cut_c; uint8_t trip; };   // warn_c/cut_c = 0: tắt
static safety_cfg_t safety = {};
static bool    otWarn  = false;
static uint8_t warnCnt = 0, cutCnt = 0;

static void loadSafety() {
  Preferences prefs;
  if (!prefs.begin("safe", false)) return;
  if (prefs.isKey("cfg")) {
    safety_cfg_t tmp;
    if (prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) safety = tmp;
  }
  prefs.end();
}

static void saveSafety() {
  Preferences prefs;
  prefs.begin("safe", false);
  prefs.putBytes("cfg", &safety, sizeof(safety));
  prefs.end();
}

static void loadDev() {
  Preferences prefs;
  if (!prefs.begin("dev", false)) return;
  if (prefs.isKey("cfg")) {
    dev_cfg_t tmp;
    if (prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) {
      tmp.name[NAME_LEN - 1] = 0;
      dev = tmp;
    }
  }
  prefs.end();
}

static void saveDev() {
  Preferences prefs;
  prefs.begin("dev", false);
  prefs.putBytes("cfg", &dev, sizeof(dev));
  prefs.end();
}

struct Parent { uint8_t mac[6]; uint8_t hops; int8_t rssi; uint32_t seen; bool valid; };
static Parent parent = {};

struct Route { bool used; uint8_t id[6]; uint8_t next[6]; };
static Route routes[8];

struct RxItem { uint8_t mac[6]; int8_t rssi; pkt_t p; };
static QueueHandle_t rxQ;

// ---------- tiện ích ----------
static bool macEq(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }
static bool isDownCmd(uint8_t t) {
  return t == MSG_CMD_RELAY || t == MSG_CFG_SET || t == MSG_SAFETY_SET || t == MSG_OTA_CMD;
}

static void ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;                 // dùng kênh hiện tại
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static void fillHdr(pkt_t &p, uint8_t type, const uint8_t *dst, uint8_t hops, uint8_t ttl) {
  memset(&p, 0, sizeof(p));
  p.h.magic = PROTO_MAGIC;
  p.h.type  = type;
  p.h.ttl   = ttl;
  p.h.hops  = hops;
  p.h.seq   = seqNo++;
  memcpy(p.h.src, myMac, 6);
  if (dst) memcpy(p.h.dst, dst, 6);
}

static bool uplinkOk() {
  return WiFi.status() == WL_CONNECTED && lastAckMs != 0 && (millis() - lastAckMs) < UPLINK_TIMEOUT_MS;
}
static bool hasParent() { return parent.valid && (millis() - parent.seen) < PARENT_TIMEOUT_MS; }
static uint8_t myHops() {
  if (uplinkOk())  return 1;
  if (hasParent()) return parent.hops + 1;
  return 255;
}

static void learnRoute(const uint8_t *id, const uint8_t *next) {
  Route *freeSlot = nullptr;
  for (auto &r : routes) {
    if (r.used && macEq(r.id, id)) { memcpy(r.next, next, 6); return; }
    if (!r.used && !freeSlot) freeSlot = &r;
  }
  if (!freeSlot) freeSlot = &routes[0];   // đầy thì ghi đè
  freeSlot->used = true;
  memcpy(freeSlot->id, id, 6);
  memcpy(freeSlot->next, next, 6);
}

static bool isChild(const uint8_t *mac) {
  for (auto &r : routes) if (r.used && macEq(r.next, mac)) return true;
  return false;
}

// ---------- gửi ----------
static void sendUdpToMaster(const pkt_t &p) {
  if (WiFi.status() != WL_CONNECTED || !udpStarted) return;
  if ((uint32_t)masterIp != 0) {
    udp.beginPacket(masterIp, UDP_PORT);
    udp.write((const uint8_t *)&p, sizeof(p));
    udp.endPacket();
  }
  if (!uplinkOk()) {   // chưa có ACK: broadcast để master tìm ra mình
    udp.beginPacket(WiFi.broadcastIP(), UDP_PORT);
    udp.write((const uint8_t *)&p, sizeof(p));
    udp.endPacket();
  }
}

static void sendToParent(const pkt_t &p) {
  if (!hasParent()) return;
  ensurePeer(parent.mac);
  esp_now_send(parent.mac, (const uint8_t *)&p, sizeof(p));
}

static void sendTelemetry() {
  pkt_t p;
  fillHdr(p, MSG_TELEMETRY, nullptr, 0, MAX_TTL);
  p.temp  = gTemp;
  p.lux   = gLux;
  p.relay = relayOn ? 1 : 0;

  // chất lượng đường lên của chính node này
  if (uplinkOk())       { p.link = LINK_WIFI;   p.rssi = WiFi.RSSI(); }
  else if (hasParent()) { p.link = LINK_ESPNOW; p.rssi = parent.rssi; }
  else                  { p.link = LINK_NONE;   p.rssi = 0; }

  // thông tin thiết bị
  p.flags = dev.configured ? 0 : FLAG_NEW;
  if (otWarn)      p.flags |= FLAG_OT_WARN;
  if (safety.trip) p.flags |= FLAG_OT_TRIP;
  p.warn_c = safety.warn_c;
  p.cut_c  = safety.cut_c;
  p.fw_ver = FW_VERSION;
  p.area  = dev.area;
  p.dtype = dev.dtype;
  memcpy(p.name, dev.name, NAME_LEN);

  sendUdpToMaster(p);                 // có Wi-Fi thì gửi (và chờ ACK)
  if (!uplinkOk()) sendToParent(p);   // không có đường lên master thì nhờ node hàng xóm
}

static void sendBeacon() {
  uint8_t h = myHops();
  if (h >= MAX_HOPS) return;
  pkt_t b;
  fillHdr(b, MSG_BEACON, BCAST, h, 1);
  esp_now_send(BCAST, (const uint8_t *)&b, sizeof(b));
}

// chuyển tiếp gói của node con lên master
static void forwardUp(pkt_t p, const uint8_t *from) {
  if (p.h.ttl == 0) return;
  p.h.ttl--;
  p.h.hops++;
  if (uplinkOk()) sendUdpToMaster(p);
  else if (hasParent() && !macEq(parent.mac, from)) sendToParent(p);
}

// chuyển lệnh từ master xuống node con
static void routeDown(pkt_t p) {
  if (p.h.ttl == 0) return;
  for (auto &r : routes) {
    if (r.used && macEq(r.id, p.h.dst)) {
      p.h.ttl--;
      ensurePeer(r.next);
      esp_now_send(r.next, (const uint8_t *)&p, sizeof(p));
      return;
    }
  }
}

// ---------- relay ----------
static void applyRelay(bool on) {
  if (on && safety.trip) {            // đang khóa do quá nhiệt: từ chối bật
    Serial.println("RELAY: tu choi BAT - dang khoa do qua nhiet");
    sendTelemetry();
    return;
  }
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
  Serial.printf("RELAY -> %s\n", on ? "ON" : "OFF");
  sendTelemetry();                    // báo lại trạng thái ngay
}

// Chạy mỗi SAFETY_MS ngay trong loop(): không cần Wi-Fi / ESP-NOW / master
static void safetyCheck() {
  float t = gTemp;
  bool changed = false;

  // --- cảnh báo (có trễ để không nhấp nháy) ---
  if (safety.warn_c > 0) {
    if (!otWarn) {
      if (t >= safety.warn_c) {
        if (++warnCnt >= OT_CONFIRM_COUNT) { otWarn = true; changed = true; }
      } else {
        warnCnt = 0;
      }
    } else if (t < safety.warn_c - OT_WARN_HYST) {
      otWarn = false; warnCnt = 0; changed = true;
    }
  } else if (otWarn) {
    otWarn = false; warnCnt = 0; changed = true;
  }

  // --- ngắt relay, khóa chốt (lưu NVS nên mất điện vẫn giữ khóa) ---
  // Đọc cảm biến hỏng/hở mạch (nhiệt độ chạm T_MAX) cũng sẽ ngắt -> an toàn khi cảm biến lỗi
  if (safety.cut_c > 0 && !safety.trip) {
    if (t >= safety.cut_c) {
      if (++cutCnt >= OT_CONFIRM_COUNT) {
        cutCnt = 0;
        safety.trip = 1;
        saveSafety();
        Serial.printf("!!! QUA NHIET %.1f C >= %d C -> NGAT RELAY, KHOA\n", t, (int)safety.cut_c);
        applyRelay(false);            // tắt relay + gửi telemetry ngay (đã kèm cờ TRIP)
        changed = false;
      }
    } else {
      cutCnt = 0;
    }
  }

  if (changed) sendTelemetry();
}

static void applySafety(const pkt_t &p) {
  int16_t w = p.warn_c, c = p.cut_c;
  bool valid = w >= 0 && c >= 0 && w <= 150 && c <= 150 && (w == 0 || c == 0 || w < c);
  if (valid) {
    safety.warn_c = w;
    safety.cut_c  = c;
    warnCnt = 0; cutCnt = 0;
  }
  if ((p.flags & FLAG_CLEAR_TRIP) && safety.trip) {
    float lim = safety.cut_c > 0 ? safety.cut_c - OT_CLEAR_MARGIN : 1e9f;
    if (gTemp < lim) {
      safety.trip = 0;
      cutCnt = 0;
      Serial.println("SAFETY: da mo khoa (relay van TAT, hay bat lai thu cong)");
    } else {
      Serial.printf("SAFETY: tu choi mo khoa, nhiet do %.1f C con cao\n", gTemp);
    }
  }
  saveSafety();
  Serial.printf("SAFETY: warn=%d cut=%d trip=%d\n", (int)safety.warn_c, (int)safety.cut_c, (int)safety.trip);
  sendTelemetry();
}

// ---------- OTA: tự kết nối server, tải firmware khi có lệnh từ app ----------
struct OtaEvt { uint8_t state, pct, err; uint32_t ver; };
static QueueHandle_t   otaQ = nullptr;
static volatile bool   otaBusy = false, otaCheckOnly = false;
static uint8_t         otaLastPct = 0;
static uint32_t        otaNewVer = 0;

// Task OTA chỉ đẩy sự kiện vào hàng đợi; loop() mới là nơi gửi (WiFiUDP không an toàn đa luồng)
static void otaEvt(uint8_t st, uint8_t pct, uint8_t err, uint32_t ver) {
  if (!otaQ) return;
  OtaEvt e = {st, pct, err, ver};
  xQueueSend(otaQ, &e, 0);
}

static void otaDrain() {
  OtaEvt e;
  while (otaQ && xQueueReceive(otaQ, &e, 0) == pdTRUE) {
    pkt_t p;
    fillHdr(p, MSG_OTA_STATUS, nullptr, 0, MAX_TTL);
    p.fw_ver    = FW_VERSION;
    p.fw_new    = e.ver;
    p.ota_state = e.state;
    p.ota_pct   = e.pct;
    p.ota_err   = e.err;
    Serial.printf("OTA: state=%d pct=%d err=%d\n", e.state, e.pct, e.err);
    sendUdpToMaster(p);
    if (!uplinkOk()) sendToParent(p);
  }
}

// PHP json_encode viết "https:\/\/host\/..." (escape dấu /), nên phải bỏ dấu '\' khi đọc chuỗi
static const char *jsonVal(const char *j, const char *key) {
  char pat[32];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(j, pat);
  if (!p) return nullptr;
  p += strlen(pat);
  while (*p == ' ' || *p == ':' || *p == '\t' || *p == '\n' || *p == '\r') p++;
  return p;
}

static bool jsonStr(const char *j, const char *key, char *out, size_t cap) {
  const char *p = jsonVal(j, key);
  if (!p || *p != '"') return false;
  p++;
  size_t n = 0;
  while (*p && *p != '"' && n + 1 < cap) {
    if (*p == '\\' && p[1]) p++;       // \/ -> /   \\ -> \   \" -> "
    out[n++] = *p++;
  }
  out[n] = 0;
  return *p == '"';
}

static void otaTls(WiFiClientSecure &c) {
#ifdef OTA_ROOT_CA
  c.setCACert(OTA_ROOT_CA);            // kiểm tra chứng chỉ server (khuyến nghị)
#else
  c.setInsecure();                     // CHƯA kiểm tra chứng chỉ -> nên định nghĩa OTA_ROOT_CA (PEM)
#endif
}

static void otaProgress(int cur, int total) {
  if (total <= 0) return;
  int pct = (int)((int64_t)cur * 100 / total);
  if (pct >= otaLastPct + 10) {
    otaLastPct = (uint8_t)(pct - pct % 10);
    otaEvt(OTA_ST_DOWNLOADING, (uint8_t)pct, OTA_ERR_NONE, otaNewVer);
  }
}

static void otaRun(bool checkOnly) {
  otaEvt(OTA_ST_CHECKING, 0, OTA_ERR_NONE, 0);
  if (WiFi.status() != WL_CONNECTED) {
    otaEvt(OTA_ST_FAILED, 0, OTA_ERR_NO_WIFI, 0);      // node chỉ có đường ESP-NOW thì không tự tải được
    return;
  }

  char url[200];
  snprintf(url, sizeof(url), "%s?type=node&v=%lu", OTA_CHECK_URL, (unsigned long)FW_VERSION);
  String body;
  {
    WiFiClientSecure c;
    otaTls(c);
    HTTPClient http;
    http.setTimeout(10000);
    if (!http.begin(c, url)) { otaEvt(OTA_ST_FAILED, 0, OTA_ERR_CHECK, 0); return; }
    int code = http.GET();
    if (code != 200) {
      Serial.printf("OTA: check HTTP %d\n", code);
      http.end();
      otaEvt(OTA_ST_FAILED, 0, OTA_ERR_CHECK, 0);
      return;
    }
    body = http.getString();
    http.end();
  }

  const char *hv = jsonVal(body.c_str(), "has_update");
  bool has = hv && strncmp(hv, "true", 4) == 0;
  if (!has) { otaEvt(OTA_ST_UP_TO_DATE, 0, OTA_ERR_NONE, FW_VERSION); return; }

  const char *lv = jsonVal(body.c_str(), "latest_version");
  otaNewVer = lv ? (uint32_t)strtoul(lv, nullptr, 10) : 0;
  char bin[200];
  if (!jsonStr(body.c_str(), "bin_url", bin, sizeof(bin)) || !bin[0]) {
    otaEvt(OTA_ST_FAILED, 0, OTA_ERR_CHECK, otaNewVer);
    return;
  }
  if (checkOnly) { otaEvt(OTA_ST_AVAILABLE, 0, OTA_ERR_NONE, otaNewVer); return; }

  Serial.printf("OTA: tai %s (v%lu -> v%lu)\n", bin, (unsigned long)FW_VERSION, (unsigned long)otaNewVer);
  otaLastPct = 0;
  otaEvt(OTA_ST_DOWNLOADING, 0, OTA_ERR_NONE, otaNewVer);

  WiFiClientSecure c;
  otaTls(c);
  httpUpdate.rebootOnUpdate(false);                    // tự khởi động lại sau khi đã báo REBOOTING
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.onProgress(otaProgress);
  t_httpUpdate_return r = httpUpdate.update(c, bin);

  if (r == HTTP_UPDATE_OK) {
    otaEvt(OTA_ST_REBOOTING, 100, OTA_ERR_NONE, otaNewVer);
    vTaskDelay(pdMS_TO_TICKS(2000));                   // chờ loop() gửi xong trạng thái
    ESP.restart();
  } else {
    Serial.printf("OTA loi (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    uint8_t err = (httpUpdate.getLastError() == HTTP_UE_TOO_LESS_SPACE) ? OTA_ERR_FLASH : OTA_ERR_DOWNLOAD;
    otaEvt(OTA_ST_FAILED, 0, err, otaNewVer);
  }
}

// Chạy trong task riêng để loop() (và bảo vệ quá nhiệt) vẫn hoạt động suốt lúc tải
static void otaTask(void *) {
  otaRun(otaCheckOnly);
  otaBusy = false;
  vTaskDelete(nullptr);
}

static void startOta(bool checkOnly) {
  if (otaBusy) { otaEvt(OTA_ST_BUSY, otaLastPct, OTA_ERR_NONE, otaNewVer); return; }
  otaBusy = true;
  otaCheckOnly = checkOnly;
  if (xTaskCreate(otaTask, "ota", 12288, nullptr, 1, nullptr) != pdPASS) {
    otaBusy = false;
    otaEvt(OTA_ST_FAILED, 0, OTA_ERR_DOWNLOAD, 0);
  }
}

static void applyCfg(const pkt_t &p) {
  dev.configured = 1;
  dev.area  = p.area;
  dev.dtype = p.dtype;
  memcpy(dev.name, p.name, NAME_LEN);
  dev.name[NAME_LEN - 1] = 0;
  saveDev();
  Serial.printf("CFG: name=%s area=%d type=%d\n", dev.name, dev.area, dev.dtype);
  sendTelemetry();                    // báo lại ngay để app thấy đã áp dụng
}

static void handleCmd(const pkt_t &p) {
  if (macEq(p.h.dst, myMac)) {
    if (p.h.type == MSG_CMD_RELAY)     applyRelay(p.relay != 0);
    else if (p.h.type == MSG_CFG_SET)  applyCfg(p);
    else if (p.h.type == MSG_SAFETY_SET) applySafety(p);
    else if (p.h.type == MSG_OTA_CMD)    startOta((p.flags & FLAG_OTA_CHECK_ONLY) != 0);
  } else {
    routeDown(p);
  }
}

// ---------- nhận ESP-NOW ----------
static void pushRx(const uint8_t *mac, int8_t rssi, const uint8_t *data, int len) {
  if (len != (int)sizeof(pkt_t) || !rxQ) return;
  RxItem it;
  memcpy(it.mac, mac, 6);
  it.rssi = rssi;
  memcpy(&it.p, data, sizeof(pkt_t));
  xQueueSend(rxQ, &it, 0);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  pushRx(info->src_addr, info->rx_ctrl ? info->rx_ctrl->rssi : -60, data, len);
}
#else
static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  pushRx(mac, -60, data, len);
}
#endif

static void handleEspNow(const RxItem &it) {
  const pkt_t &p = it.p;
  if (p.h.magic != PROTO_MAGIC) return;

  switch (p.h.type) {
    case MSG_BEACON: {
      if (uplinkOk() || isChild(it.mac)) return;     // đã có đường lên, hoặc đó là node con của mình
      uint8_t h = p.h.hops;
      if (h >= MAX_HOPS) return;
      bool better = !hasParent() || macEq(it.mac, parent.mac) || h < parent.hops ||
                    (h == parent.hops && it.rssi > parent.rssi + 8);
      if (better) {
        memcpy(parent.mac, it.mac, 6);
        parent.hops = h;
        parent.rssi = it.rssi;
        parent.seen = millis();
        parent.valid = true;
      }
      break;
    }
    case MSG_OTA_STATUS:                 // trạng thái OTA của node con -> chuyển lên như telemetry
    case MSG_TELEMETRY:                  // từ node con cần chuyển tiếp
      learnRoute(p.h.src, it.mac);
      forwardUp(p, it.mac);
      break;
    case MSG_CMD_RELAY:
    case MSG_CFG_SET:
    case MSG_SAFETY_SET:
    case MSG_OTA_CMD:
      handleCmd(p);
      break;
    default: break;
  }
}

// ---------- nhận UDP ----------
static void pollUdp() {
  if (!udpStarted) return;
  int n = udp.parsePacket();
  while (n > 0) {
    if (n == (int)sizeof(pkt_t)) {
      pkt_t p;
      udp.read((uint8_t *)&p, sizeof(p));
      if (p.h.magic == PROTO_MAGIC) {
        if (p.h.type == MSG_ACK) {
          lastAckMs = millis();
          if (lastAckMs == 0) lastAckMs = 1;
          masterIp = udp.remoteIP();
        } else if (isDownCmd(p.h.type)) {
          handleCmd(p);
        }
      }
    } else {
      udp.flush();
    }
    n = udp.parsePacket();
  }
}

// ---------- BLE provisioning ----------
class ProvCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.size() == sizeof(prov_blob_t)) {
      prov_blob_t b;
      memcpy(&b, v.data(), sizeof(b));
      if (b.magic == PROV_MAGIC) {
        b.ssid[sizeof(b.ssid) - 1] = 0;
        b.pass[sizeof(b.pass) - 1] = 0;
        pendingBlob = b;
        provPending = true;
      }
    }
  }
};

static void bleStart() {
  char name[16];
  snprintf(name, sizeof(name), "%s%02X%02X%02X", BLE_NAME_PREFIX, myMac[3], myMac[4], myMac[5]);
  NimBLEDevice::init(name);
  NimBLEServer *server = NimBLEDevice::createServer();
  NimBLEService *svc = server->createService(PROV_SVC_UUID_STR);
  NimBLECharacteristic *chr = svc->createCharacteristic(PROV_CHR_UUID_STR, NIMBLE_PROPERTY::WRITE);
  chr->setCallbacks(new ProvCb());
  svc->start();

  // Chỉ quảng bá flags + tên (vừa gói 31 byte) để master nhận ra qua tiền tố "SHN-"
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad;
  ad.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  ad.setName(name);
  adv->setAdvertisementData(ad);
  adv->setScanResponse(false);
  adv->start();
  bleOn = true;
  Serial.printf("BLE advertising: %s\n", name);
}

static bool loadProv() {
  Preferences prefs;
  if (!prefs.begin("prov", false)) {
    return false;
  }
  // Kiểm tra key tồn tại trước khi đọc để tránh log lỗi NVS
  if (!prefs.isKey("blob")) {
    prefs.end();
    return false;
  }
  size_t n = prefs.getBytes("blob", &prov, sizeof(prov));
  prefs.end();
  return (n == sizeof(prov) && prov.magic == PROV_MAGIC);
}

static void saveProvAndRestart() {
  Preferences prefs;
  prefs.begin("prov", false);
  prefs.putBytes("blob", &pendingBlob, sizeof(pendingBlob));
  prefs.end();
  Serial.printf("Nhan cau hinh tu master: SSID=%s ch=%d -> khoi dong lai\n", pendingBlob.ssid, pendingBlob.channel);
  delay(1500);                         // chờ BLE trả lời xong
  ESP.restart();
}

// ---------- đọc cảm biến ----------
static void readTemp(bool dbg) {
  float vAdc = readAdcMilliVolts(NTC_PIN, 30) - 25;
  float rNtc = calculateRntc(vAdc);
  gTemp = calculateTemperature(rNtc);
  if (dbg) {
    Serial.printf("VADC: %.1f mV | RNTC: %.2f Ohm | Temp: %.2f C\n", vAdc, rNtc, gTemp);
  }
}

static void readSensors() {
  readTemp(true);
  gLux = calculateLux(tb(LIGHT_PIN));
}

void setup() {
  Serial.begin(115200);

  // 1. Khởi tạo phân vùng NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 2. Đọc MAC phần cứng từ eFuse
  esp_read_mac(myMac, ESP_MAC_WIFI_STA);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, !RELAY_ACTIVE_LEVEL);

  // 3. Kiểm tra xem đã có cấu hình trong Flash chưa
  provValid = loadProv();
  loadSafety();                        // ngưỡng quá nhiệt + trạng thái khóa (relay luôn khởi động ở trạng thái TẮT)
  loadDev();                           // tên / khu vực / loại thiết bị đã đặt (nếu có)

  if (provValid) {
    // --- ĐÃ CÓ CẤU HÌNH: CHỈ CHẠY WI-FI & ESP-NOW (KHÔNG BẬT BLE) ---
    Serial.printf("Node MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
    Serial.printf("Da co cau hinh: SSID=%s, kenh %d\n", prov.ssid, prov.channel);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // An toàn vì BLE không chạy, tối ưu cho ESP-NOW
    
    masterIp = IPAddress(prov.master_ip);
    esp_wifi_set_channel(prov.channel, WIFI_SECOND_CHAN_NONE);
    WiFi.begin(prov.ssid, prov.pass, prov.channel);

    rxQ = xQueueCreate(16, sizeof(RxItem));
    otaQ = xQueueCreate(8, sizeof(OtaEvt));
    if (esp_now_init() == ESP_OK) {
      esp_now_register_recv_cb(onEspNowRecv);
      ensurePeer(BCAST);
    }
  } else {
    // --- CHƯA CÓ CẤU HÌNH: CHỈ BẬT BLE ĐỂ MASTER NẠP (KHÔNG BẬT WI-FI) ---
    Serial.printf("Node MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
    Serial.println("Chua co cau hinh -> Bat BLE cho master quet (gui lenh 'scan')...");
    bleStart();
  }
}

void loop() {
  static uint32_t lastBeacon = 0, lastTelem = 0, lastSafety = 0;
  uint32_t now = millis();

  // Nhận được cấu hình qua BLE -> lưu Flash và khởi động lại
  if (provPending) {
    saveProvAndRestart();
  }

  // Bảo vệ quá nhiệt: luôn chạy, kể cả khi mất Wi-Fi / mất master / chưa có đường lên
  if (now - lastSafety >= SAFETY_MS) {
    lastSafety = now;
    readTemp(false);
    safetyCheck();
  }

  // Chỉ chạy các tác vụ mạng khi đã có cấu hình đầy đủ
  if (provValid) {
    if (!udpStarted && WiFi.status() == WL_CONNECTED) {
      udp.begin(UDP_PORT);
      udpStarted = true;
      Serial.print("Wi-Fi OK, IP: ");
      Serial.println(WiFi.localIP());
    }
    pollUdp();
    otaDrain();

    RxItem it;
    while (xQueueReceive(rxQ, &it, 0) == pdTRUE) {
      handleEspNow(it);
    }

    if (now - lastBeacon >= BEACON_MS) {
      lastBeacon = now;
      sendBeacon();
    }

    if (now - lastTelem >= TELEMETRY_MS) {
      lastTelem = now;
      readSensors();
      sendTelemetry();
      Serial.printf("Temp: %.2f C | Lux: %.1f | Relay: %s | Wi-Fi: %s | Uplink: %s | Parent: %s\n",
                    gTemp, gLux, relayOn ? "ON" : "OFF",
                    WiFi.status() == WL_CONNECTED ? "yes" : "no",
                    uplinkOk() ? "master" : (hasParent() ? "relay" : "none"),
                    hasParent() ? "yes" : "no");
    }
  }

  delay(5);
}