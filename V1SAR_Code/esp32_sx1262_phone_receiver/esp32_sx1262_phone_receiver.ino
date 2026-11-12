#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <FS.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include "mbedtls/base64.h"

// --- diymore / Heltec ESP32-S3 LoRa V4 (863-928MHz) SX1262 Internal SPI Pins ---
#define LORA_NSS    8
#define LORA_SCK    9
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_DIO1   14

// --- Heltec / diymore LoRa V4 Front-End Module (PA/LNA Amplifier) Control Pins ---
#define FEM_VFEM    7   // Front-end LDO power supply enable
#define FEM_CSD     2   // Front-end chip enable
#define FEM_CPS_V42 46  // Mode control (LOW = LNA Receive / Bypass, HIGH = PA Transmit)
#define FEM_CPS_V43 5   // Mode control (LOW = LNA Receive / Bypass, HIGH = PA Transmit)
#define LED_PIN     35  // Onboard white status LED

// --- Phone Wi-Fi Access Point Credentials ---
const char* AP_SSID = "SAR-Drone-Cam";
const char* AP_PASS = "rescue123";

SPIClass loraSPI(HSPI);
class LoRaSX1262 : public SX1262 {
public:
  using SX1262::SX1262;
  using SX126x::getPacketStatus;
};
LoRaSX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
WebServer server(80);
bool hasFlashFS = false;

struct LevelConfig {
  uint8_t sf;
  uint32_t bwHz;
  uint8_t cr;
  bool ldro;
  const char* name;
};

// 8 Speed Levels matching Raspberry Pi Waveshare SX1262 SPI HAT
const LevelConfig LEVELS[8] = {
  {12, 125000UL, 5, true,  "LEVEL 0 (SF12/125k)"},
  {11, 125000UL, 5, true,  "LEVEL 1 (SF11/125k)"},
  {10, 125000UL, 5, false, "LEVEL 2 (SF10/125k)"},
  {9,  250000UL, 5, false, "LEVEL 3 (SF9/250k)"},
  {8,  250000UL, 5, false, "LEVEL 4 (SF8/250k)"},
  {7,  250000UL, 5, false, "LEVEL 5 (SF7/250k Fast)"},
  {7,  500000UL, 5, false, "LEVEL 6 (SF7/500k Turbo)"},
  {6,  500000UL, 5, false, "LEVEL 7 (SF6/500k Max)"}
};

int currentLevel = 5;
volatile bool rxFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void onLoRaRx() {
  rxFlag = true;
}

// --- 1. Multi-Resolution Picture Block & Full-Frame Storage (Supports up to 1280x960 = 1200 Blocks!) ---
// Uses a single 90KB static string pool with ZERO startup malloc() calls so WiFi.softAP() always has >180KB free heap!
#define MAX_CHUNKS 320
#define MAX_TILES 1200
#define TILE_POOL_SIZE 92160
#define CHUNK_STR_MAX 196
#define TILE_B64_MAX 196
#define MAX_JPG_BYTES 16384

char tilePool[TILE_POOL_SIZE];
uint32_t tilePoolHead = 0;
int32_t tileOffset[MAX_TILES];
uint8_t tileCap[MAX_TILES];
bool tileReceivedEver[MAX_TILES];
bool chunkValid[MAX_CHUNKS];
int totalChunks = 300;
int receivedChunks = 0;
int corruptedPackets = 0;
bool jpgDirty = false;
bool flashDirty = false;

uint8_t* latestJpgBuf = nullptr;
size_t latestJpgLen = 0;
uint32_t imageVersion = 0;

inline const char* getSlot(int idx) {
  if (idx < 0 || idx >= MAX_TILES) return nullptr;
  int32_t off = tileOffset[idx];
  if (off < 0 || off >= TILE_POOL_SIZE) return nullptr;
  return &tilePool[off];
}

void storeSlot(int idx, const char* s) {
  if (idx < 0 || idx >= MAX_TILES || s == nullptr) return;
  size_t len = strlen(s);
  if (len == 0 || len >= TILE_B64_MAX) return;
  uint8_t need = (uint8_t)(len + 1);

  int32_t existingOff = tileOffset[idx];
  if (existingOff >= 0 && tileCap[idx] >= need) {
    memcpy(&tilePool[existingOff], s, need);
    return;
  }

  if (tilePoolHead + need > TILE_POOL_SIZE) {
    tilePoolHead = 0;
  }
  uint32_t newStart = tilePoolHead;
  uint32_t newEnd   = newStart + need;

  for (int i = 0; i < MAX_TILES; i++) {
    int32_t o = tileOffset[i];
    if (o >= 0) {
      uint32_t oStart = (uint32_t)o;
      uint32_t oEnd   = oStart + (uint32_t)tileCap[i];
      if (newStart < oEnd && newEnd > oStart) {
        tileOffset[i] = -1;
        tileCap[i] = 0;
      }
    }
  }

  memcpy(&tilePool[newStart], s, need);
  tileOffset[idx] = (int32_t)newStart;
  tileCap[idx]    = need;
  tilePoolHead    = newEnd;
}

// --- 2. Multi-Resolution 64x16 Picture Block Grid State ---
int imgWidth = 640;
int imgHeight = 480;
int tileCols = 10;
int tileRows = 30;
int activeNumTiles = 300;
uint32_t tileSeq[MAX_TILES];
uint32_t globalTileSeq = 0;
uint32_t clearSeq = 0;
String jhParts[6];
int jhTotal = 0;
String commonTileHeaderB64 = "";

// --- 3. Live Telemetry, Constant RSSI History & Synced Drone Config ---
int detCaptureId = 0;
float detMaxConf = 0.0f;
int detPersonCount = 0;
int detJpgQuality = 62;
String detModeStr  = "SCAN_TARGET";
String detModelStr = "SSD MobileNet";
String detTimeStr  = "--:--:--";
String detGpsStr   = "Waiting for Drone GPS...";
String detNoteStr  = "Waiting for Broadcast Details...";

uint32_t heartbeatSeq = 0;
String droneZoomState = "WIDE_SEARCH";
float droneLiveConf = 0.0f;

float lastRssi = -120.0f;
float lastSnr = 0.0f;
float minRssi = 0.0f;
float maxRssi = -150.0f;
unsigned long lastPacketMs = 0;
uint32_t totalPacketsRx = 0;
uint32_t linkDropCount = 0;
bool wasDropped = false;

#define RSSI_HIST_LEN 40
int rssiHistory[RSSI_HIST_LEN];
int rssiHistCount = 0;

static const char B64_TABLE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void clearAllImageBuffers();

void setResolutionGrid(int w, int h) {
  if (w < 320 || w > 1280 || h < 240 || h > 960) return;
  int tc = w / 64;
  int tr = h / 16;
  if (tc < 1 || tr < 1 || (tc * tr) > MAX_TILES) return;
  if (w != imgWidth || h != imgHeight) {
    imgWidth = w;
    imgHeight = h;
    tileCols = tc;
    tileRows = tr;
    activeNumTiles = tc * tr;
    totalChunks = activeNumTiles;
    clearAllImageBuffers();
  }
}

void inferResolutionFromTotalTiles(int tot) {
  if (tot == 75)   setResolutionGrid(320, 240);
  if (tot == 300)  setResolutionGrid(640, 480);
  if (tot == 675)  setResolutionGrid(960, 720);
  if (tot == 1200) setResolutionGrid(1280, 960);
}

// --- Flash (LittleFS) Persistence Helpers (Deferred Until Radio Is Idle!) ---
void saveMetaAndTilesToFlash() {
  if (!hasFlashFS) return;
  File f = LittleFS.open("/meta.txt", "w");
  if (f) {
    f.println(detCaptureId);
    f.println(detMaxConf, 1);
    f.println(detPersonCount);
    f.println(detModeStr);
    f.println(detModelStr);
    f.println(detTimeStr);
    f.println(detGpsStr);
    f.println(detNoteStr);
    f.println(commonTileHeaderB64);
    f.println(imgWidth);
    f.println(imgHeight);
    f.close();
  }
  if (detModeStr != "2D_FULL" && receivedChunks > 0) {
    File tf = LittleFS.open("/tiles.log", "w");
    if (tf) {
      for (int t = 0; t < activeNumTiles && t < MAX_TILES; t++) {
        const char* slot = getSlot(t);
        if (tileSeq[t] > 0 && slot != nullptr && slot[0] != '\0') {
          int r = t / tileCols;
          int c = t % tileCols;
          tf.printf("%d:%d:%s\n", c, r, slot);
        }
      }
      tf.close();
    }
  }
}

void flushToFlashIfIdle() {
  if (!flashDirty || !hasFlashFS) return;
  if ((millis() - lastPacketMs) < 1200UL) return; // Never block SPI LoRa while a burst is arriving!
  flashDirty = false;
  saveMetaAndTilesToFlash();
}

void appendRssiToFlash(float rssi, float snr, const char* tag) {
  if (!hasFlashFS) return;
  if ((millis() - lastPacketMs) < 600UL && strcmp(tag, "ALERT") == 0) return;
  // Keep flash RSSI log under 64KB
  if (LittleFS.exists("/rssi.csv")) {
    File rf = LittleFS.open("/rssi.csv", "r");
    if (rf && rf.size() > 64000) {
      rf.close();
      LittleFS.remove("/rssi.csv");
    } else if (rf) {
      rf.close();
    }
  }
  File f = LittleFS.open("/rssi.csv", "a");
  if (f) {
    f.printf("%lu,%u,%.1f,%.1f,%s,%.1f,\"%s\"\n",
             millis() / 1000UL, totalPacketsRx, rssi, snr, tag, droneLiveConf, detGpsStr.c_str());
    f.close();
  }
}

void restoreFromFlashOnBoot() {
  if (!hasFlashFS) return;
  if (LittleFS.exists("/meta.txt")) {
    File f = LittleFS.open("/meta.txt", "r");
    if (f) {
      detCaptureId   = f.readStringUntil('\n').toInt();
      detMaxConf     = f.readStringUntil('\n').toFloat();
      detPersonCount = f.readStringUntil('\n').toInt();
      detModeStr     = f.readStringUntil('\n'); detModeStr.trim();
      detModelStr    = f.readStringUntil('\n'); detModelStr.trim();
      detTimeStr     = f.readStringUntil('\n'); detTimeStr.trim();
      detGpsStr      = f.readStringUntil('\n'); detGpsStr.trim();
      detNoteStr     = f.readStringUntil('\n'); detNoteStr.trim();
      commonTileHeaderB64 = f.readStringUntil('\n'); commonTileHeaderB64.trim();
      if (f.available()) {
        int fw = f.readStringUntil('\n').toInt();
        int fh = f.readStringUntil('\n').toInt();
        if (fw >= 320 && fh >= 240) setResolutionGrid(fw, fh);
      }
      f.close();
    }
  }
  if (LittleFS.exists("/tiles.log")) {
    File tf = LittleFS.open("/tiles.log", "r");
    if (tf) {
      totalChunks = activeNumTiles;
      while (tf.available()) {
        String ln = tf.readStringUntil('\n');
        ln.trim();
        int p1 = ln.indexOf(':');
        int p2 = (p1 > 0) ? ln.indexOf(':', p1 + 1) : -1;
        if (p1 > 0 && p2 > 0) {
          int c = ln.substring(0, p1).toInt();
          int r = ln.substring(p1 + 1, p2).toInt();
          String b64 = ln.substring(p2 + 1);
          if (c >= 0 && c < tileCols && r >= 0 && r < tileRows && b64.length() < TILE_B64_MAX) {
            int tIdx = r * tileCols + c;
            if (!tileReceivedEver[tIdx]) {
              tileReceivedEver[tIdx] = true;
              receivedChunks++;
            }
            storeSlot(tIdx, b64.c_str());
            globalTileSeq++;
            tileSeq[tIdx] = globalTileSeq;
          }
        }
      }
      tf.close();
    }
  }
}

void recordRssiSample(float rssiVal) {
  int rInt = (int)round(rssiVal);
  if (totalPacketsRx == 1 || rssiVal < minRssi) minRssi = rssiVal;
  if (totalPacketsRx == 1 || rssiVal > maxRssi) maxRssi = rssiVal;

  if (rssiHistCount < RSSI_HIST_LEN) {
    rssiHistory[rssiHistCount++] = rInt;
  } else {
    for (int i = 1; i < RSSI_HIST_LEN; i++) {
      rssiHistory[i - 1] = rssiHistory[i];
    }
    rssiHistory[RSSI_HIST_LEN - 1] = rInt;
  }
}

void computeChecksum2(const char* str, int len, char* out2) {
  uint16_t crc = 0xACE1;
  for (int i = 0; i < len; i++) {
    crc ^= (((uint16_t)(uint8_t)str[i]) << 4);
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000)
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
      else
        crc = (crc << 1) & 0xFFFF;
    }
  }
  out2[0] = B64_TABLE[(crc >> 6) & 0x3F];
  out2[1] = B64_TABLE[crc & 0x3F];
  out2[2] = '\0';
}

void clearAllImageBuffers() {
  for (int i = 0; i < MAX_CHUNKS; i++) {
    chunkValid[i] = false;
  }
  tilePoolHead = 0;
  for (int t = 0; t < MAX_TILES; t++) {
    tileSeq[t] = 0;
    tileOffset[t] = -1;
    tileCap[t] = 0;
    tileReceivedEver[t] = false;
  }
  receivedChunks = 0;
  jpgDirty = false;
  globalTileSeq++;
}

void clearImageAndFlash() {
  clearAllImageBuffers();
  totalChunks = activeNumTiles;
  latestJpgLen = 0;
  imageVersion++;
  clearSeq++;
  flashDirty = false;
  if (hasFlashFS) {
    LittleFS.remove("/tiles.log");
    LittleFS.remove("/meta.txt");
  }
}

// Rebuilds progressive 2D JPEG ONLY when radio is idle (never blocks incoming LoRa packets!)
void rebuildProgressiveJpegIfIdle() {
  if (!jpgDirty || totalChunks <= 0) return;
  if ((millis() - lastPacketMs) < 160 && receivedChunks < totalChunks) return;
  jpgDirty = false;
  if (latestJpgBuf == nullptr) {
    if (ESP.getFreeHeap() > 75000) {
      latestJpgBuf = (uint8_t*)malloc(MAX_JPG_BYTES);
    }
    if (latestJpgBuf == nullptr) return;
  }

  static String b64Joined;
  b64Joined = "";
  b64Joined.reserve(totalChunks * 190 + 8);

  int contiguousCount = 0;
  for (int i = 0; i < totalChunks && i < MAX_CHUNKS; i++) {
    const char* slot = getSlot(i);
    if (chunkValid[i] && slot != nullptr) {
      b64Joined += slot;
      contiguousCount++;
    } else {
      break;
    }
  }

  if (contiguousCount < 2 && receivedChunks < totalChunks) return;

  while ((b64Joined.length() % 4) != 0) {
    b64Joined += "=";
  }

  size_t outLen = 0;
  int rc = mbedtls_base64_decode(
    latestJpgBuf,
    MAX_JPG_BYTES - 4,
    &outLen,
    (const unsigned char*)b64Joined.c_str(),
    b64Joined.length()
  );

  if (rc == 0 && outLen > 16) {
    if (latestJpgBuf[outLen - 2] != 0xFF || latestJpgBuf[outLen - 1] != 0xD9) {
      latestJpgBuf[outLen++] = 0xFF;
      latestJpgBuf[outLen++] = 0xD9;
    }
    latestJpgLen = outLen;
    imageVersion++;
  }
}

void applyLoRaLevel(int lvl) {
  if (lvl < 0 || lvl > 7) return;
  currentLevel = lvl;
  radio.standby();
  radio.setSpreadingFactor(LEVELS[lvl].sf);
  radio.setBandwidth(LEVELS[lvl].bwHz);
  radio.setCodingRate(LEVELS[lvl].cr);
  radio.forceLDRO(LEVELS[lvl].ldro);
  radio.startReceive();
  Serial.printf("[LoRa RX] Switched to %s @ 868.0 MHz\r\n", LEVELS[lvl].name);
}

void processIncomingLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "CMD:CLEAR") {
    clearImageAndFlash();
    return;
  }

  if (line.startsWith("CMD:LEVEL:")) {
    int newLvl = line.substring(10).toInt();
    if (newLvl >= 0 && newLvl <= 7 && newLvl != currentLevel) {
      applyLoRaLevel(newLvl);
    }
    return;
  }

  // 0. Instant Over-The-Air Config Sync Packet from Pi: CFG:<level>:<txMode>:<aiModel>:<confirmThresh>:<txW>:<txH>
  if (line.startsWith("CFG:")) {
    int p1 = line.indexOf(':', 4);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    int p4 = (p3 > 0) ? line.indexOf(':', p3 + 1) : -1;
    int p5 = (p4 > 0) ? line.indexOf(':', p4 + 1) : -1;
    if (p1 > 0 && p2 > 0 && p3 > 0) {
      int cfgLvl = line.substring(4, p1).toInt();
      detModeStr  = line.substring(p1 + 1, p2);
      detModelStr = line.substring(p2 + 1, p3);
      if (p4 > 0 && p5 > 0) {
        int cw = line.substring(p4 + 1, p5).toInt();
        int ch = line.substring(p5 + 1).toInt();
        setResolutionGrid(cw, ch);
      }
      if (cfgLvl >= 0 && cfgLvl <= 7 && cfgLvl != currentLevel) {
        applyLoRaLevel(cfgLvl);
      }
      flashDirty = true;
      Serial.printf("[CFG SYNC] Mode: %s | Model: %s | Res: %dx%d | Level: %d\r\n",
                    detModeStr.c_str(), detModelStr.c_str(), imgWidth, imgHeight, currentLevel);
    }
    return;
  }

  // 1. Constant 2.5-Second Heartbeat Beacon: H:<seq>:<zoomState>:<liveConf>:<txMode>:<aiModel>:<txW>:<txH>:<gpsStr>
  if (line.startsWith("H:")) {
    int p1 = line.indexOf(':', 2);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    int p4 = (p3 > 0) ? line.indexOf(':', p3 + 1) : -1;
    int p5 = (p4 > 0) ? line.indexOf(':', p4 + 1) : -1;
    int p6 = (p5 > 0) ? line.indexOf(':', p5 + 1) : -1;
    int p7 = (p6 > 0) ? line.indexOf(':', p6 + 1) : -1;
    if (p1 > 0 && p2 > 0 && p3 > 0) {
      heartbeatSeq = line.substring(2, p1).toInt();
      droneZoomState = line.substring(p1 + 1, p2);
      if (p6 > 0 && p7 > 0) {
        droneLiveConf = line.substring(p2 + 1, p3).toFloat();
        detModeStr    = line.substring(p3 + 1, p4);
        detModelStr   = line.substring(p4 + 1, p5);
        int hw        = line.substring(p5 + 1, p6).toInt();
        int hh        = line.substring(p6 + 1, p7).toInt();
        setResolutionGrid(hw, hh);
        String g      = line.substring(p7 + 1);
        if (g.length() > 0) detGpsStr = g;
      } else if (p4 > 0 && p5 > 0) {
        droneLiveConf = line.substring(p2 + 1, p3).toFloat();
        detModeStr    = line.substring(p3 + 1, p4);
        detModelStr   = line.substring(p4 + 1, p5);
        String g      = line.substring(p5 + 1);
        if (g.length() > 0) detGpsStr = g;
      } else {
        droneLiveConf = line.substring(p2 + 1, p3).toFloat();
        String g      = line.substring(p3 + 1);
        if (g.length() > 0) detGpsStr = g;
      }
      appendRssiToFlash(lastRssi, lastSnr, "HB");
    }
    return;
  }

  // 2. AI Person Detection Alert Header: D:<captureId>:<maxConfPct>:<personCount>:<totalUnits>:<jpegQuality>[:<txW>:<txH>]
  if (line.startsWith("D:")) {
    int p1 = line.indexOf(':', 2);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    int p4 = (p3 > 0) ? line.indexOf(':', p3 + 1) : -1;
    int p5 = (p4 > 0) ? line.indexOf(':', p4 + 1) : -1;
    int p6 = (p5 > 0) ? line.indexOf(':', p5 + 1) : -1;
    if (p1 > 0 && p2 > 0 && p3 > 0 && p4 > 0) {
      int newId = line.substring(2, p1).toInt();
      float conf = line.substring(p1 + 1, p2).toFloat();
      int pCount = line.substring(p2 + 1, p3).toInt();
      int tot = line.substring(p3 + 1, p4).toInt();
      int jpgQ = (p5 > 0) ? line.substring(p4 + 1, p5).toInt() : line.substring(p4 + 1).toInt();

      if (p5 > 0 && p6 > 0) {
        int dw = line.substring(p5 + 1, p6).toInt();
        int dh = line.substring(p6 + 1).toInt();
        setResolutionGrid(dw, dh);
      } else if (detModeStr != "2D_FULL") {
        inferResolutionFromTotalTiles(tot);
      }

      if (newId != detCaptureId || tot != totalChunks) {
        detCaptureId = newId;
        totalChunks = (detModeStr == "2D_FULL") ? min(tot, MAX_CHUNKS) : min(tot, MAX_TILES);
        clearAllImageBuffers();
      }
      detMaxConf = conf;
      detPersonCount = pCount;
      detJpgQuality = jpgQ;
      flashDirty = true;
    }
    return;
  }

  // 3. Broadcast Metadata, Mode, GPS & Notes Packet: M:<captureId>:<modeCode>:<timeStr>:<gpsStr>:<noteStr>
  if (line.startsWith("M:")) {
    int p1 = line.indexOf(':', 2);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    int p4 = (p3 > 0) ? line.indexOf(':', p3 + 1) : -1;
    if (p1 > 0 && p2 > 0 && p3 > 0 && p4 > 0) {
      detModeStr = line.substring(p1 + 1, p2);
      detTimeStr = line.substring(p2 + 1, p3);
      detGpsStr  = line.substring(p3 + 1, p4);
      detNoteStr = line.substring(p4 + 1);
      flashDirty = true;
    }
    return;
  }

  // 4. Common 64x16 Picture Block JPEG Header Packet: JH:<idx>:<tot>:<b64>
  if (line.startsWith("JH:")) {
    int p1 = line.indexOf(':', 3);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    if (p1 > 0 && p2 > 0) {
      int hIdx = line.substring(3, p1).toInt();
      int hTot = line.substring(p1 + 1, p2).toInt();
      String hB64 = line.substring(p2 + 1);
      if (hIdx >= 0 && hIdx < 6 && hTot > 0 && hTot <= 6) {
        jhParts[hIdx] = hB64;
        jhTotal = hTot;
        String joined = "";
        bool allOk = true;
        for (int i = 0; i < jhTotal; i++) {
          if (jhParts[i].length() == 0) { allOk = false; break; }
          joined += jhParts[i];
        }
        if (allOk && joined != commonTileHeaderB64) {
          commonTileHeaderB64 = joined;
          flashDirty = true;
        }
      }
    }
    return;
  }

  // 5. 64x16 Picture Block Packet: J<tCols>:<col>:<row>:<chk2>:<scanB64> (or legacy J:<col>:<row>:<chk2>:<scanB64>)
  if (line.startsWith("J") && !line.startsWith("JH:")) {
    int p0 = line.indexOf(':');
    int p1 = (p0 > 0) ? line.indexOf(':', p0 + 1) : -1;
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    if (p0 >= 1 && p1 > 0 && p2 > 0 && p3 > 0) {
      // Check if resolution cols (5, 10, 15, 20) is encoded in J5:, J10:, J15:, J20:
      if (p0 > 1) {
        int pktCols = line.substring(1, p0).toInt();
        if (pktCols == 5)       setResolutionGrid(320, 240);
        else if (pktCols == 10) setResolutionGrid(640, 480);
        else if (pktCols == 15) setResolutionGrid(960, 720);
        else if (pktCols == 20) setResolutionGrid(1280, 960);
      }

      int col = line.substring(p0 + 1, p1).toInt();
      int row = line.substring(p1 + 1, p2).toInt();
      String chkStr = line.substring(p2 + 1, p3);
      String scanB64 = line.substring(p3 + 1);

      if (col < 0 || col >= tileCols || row < 0 || row >= tileRows) return;
      if (scanB64.length() == 0 || scanB64.length() >= TILE_B64_MAX) return;

      char expectedChk[3];
      computeChecksum2(scanB64.c_str(), scanB64.length(), expectedChk);
      if (chkStr != expectedChk) {
        corruptedPackets++;
        return;
      }

      if (totalChunks == 0) totalChunks = activeNumTiles;
      int tIdx = row * tileCols + col;
      if (!tileReceivedEver[tIdx]) {
        tileReceivedEver[tIdx] = true;
        receivedChunks++;
        flashDirty = true;
      }
      storeSlot(tIdx, scanB64.c_str());
      globalTileSeq++;
      tileSeq[tIdx] = globalTileSeq;
    }
    return;
  }

  // 6. Full-Frame 2D Progressive JPEG Chunk Packet: F:<idx>:<total>:<chk2>:<b64Chunk>
  if (line.startsWith("F:")) {
    int p1 = line.indexOf(':', 2);
    int p2 = (p1 > 0) ? line.indexOf(':', p1 + 1) : -1;
    int p3 = (p2 > 0) ? line.indexOf(':', p2 + 1) : -1;
    if (p1 > 0 && p2 > 0 && p3 > 0) {
      int idx = line.substring(2, p1).toInt();
      int tot = line.substring(p1 + 1, p2).toInt();
      String chkStr = line.substring(p2 + 1, p3);
      String b64Chunk = line.substring(p3 + 1);

      if (tot <= 0 || tot > MAX_CHUNKS || idx < 0 || idx >= tot) return;
      if (b64Chunk.length() == 0 || b64Chunk.length() >= CHUNK_STR_MAX) return;

      char expectedChk[3];
      computeChecksum2(b64Chunk.c_str(), b64Chunk.length(), expectedChk);
      if (chkStr != expectedChk) {
        corruptedPackets++;
        return;
      }

      if (tot != totalChunks) {
        totalChunks = tot;
        clearAllImageBuffers();
      }

      if (!chunkValid[idx]) {
        storeSlot(idx, b64Chunk.c_str());
        chunkValid[idx] = true;
        receivedChunks++;
        jpgDirty = true; // Deferred decode when radio is idle so we NEVER drop packets!
      }
    }
    return;
  }
}

// --- Embedded DJI Pilot 2 / Auterion Mission Control Ground Receiver Web UI ---
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>SAR Ground Receiver</title>
<style>
  :root {
    --bg: #111317;
    --card: #1a1d24;
    --card-alt: #212630;
    --border: #2a2f3a;
    --blue: #2563eb;
    --blue-hi: #3b82f6;
    --green: #10b981;
    --amber: #f59e0b;
    --red: #ef4444;
    --text: #f3f4f6;
    --sub: #9ca3af;
  }
  * {
    box-sizing: border-box; margin: 0; padding: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-variant-numeric: tabular-nums;
  }
  html, body { width: 100%; max-width: 100%; overflow-x: hidden; background: var(--bg); color: var(--text); font-size: 13px; }
  body { padding: 8px; max-width: 720px; margin: 0 auto; }

  /* Top Status Bar */
  .header {
    display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;
    background: var(--card); border: 1px solid var(--border); border-radius: 8px;
    padding: 8px 12px; margin-bottom: 8px;
  }
  .brand { display: flex; align-items: center; gap: 8px; font-weight: 600; font-size: 14px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--green); box-shadow: 0 0 8px var(--green); }
  .dot.offline { background: var(--red); box-shadow: 0 0 8px var(--red); }
  .rf-pills { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
  .pill {
    background: var(--card-alt); border: 1px solid var(--border); border-radius: 6px;
    padding: 3px 8px; font-size: 11px; font-weight: 600; color: var(--text); white-space: nowrap;
  }
  .pill span { color: var(--sub); font-weight: 500; margin-right: 3px; }
  .sig-bars { display: inline-flex; align-items: flex-end; gap: 2px; height: 12px; margin-right: 4px; vertical-align: middle; }
  .sig-bars i { width: 3px; background: #374151; border-radius: 1px; display: inline-block; }
  .sig-bars i.on { background: var(--green); }
  .sig-bars.warn i.on { background: var(--amber); }
  .sig-bars.bad i.on { background: var(--red); }

  /* Video Viewport with Translucent Glass HUD Overlays */
  .viewport-wrap {
    position: relative; width: 100%; aspect-ratio: 4/3; background: #090b0e;
    border: 1px solid var(--border); border-radius: 8px; overflow: hidden; margin-bottom: 8px;
  }
  #viewCanvas { width: 100%; height: 100%; display: block; }
  .hud-top {
    position: absolute; top: 8px; left: 8px; right: 8px;
    display: flex; justify-content: space-between; align-items: flex-start; gap: 6px; pointer-events: none;
  }
  .hud-bottom {
    position: absolute; bottom: 10px; left: 8px; right: 8px;
    display: flex; justify-content: space-between; align-items: center; gap: 6px; pointer-events: none;
  }
  .hud-chip {
    background: rgba(15, 18, 24, 0.78); backdrop-filter: blur(6px); -webkit-backdrop-filter: blur(6px);
    border: 1px solid rgba(255, 255, 255, 0.12); border-radius: 6px;
    padding: 4px 9px; font-size: 11px; font-weight: 600; color: #f9fafb;
    max-width: 100%; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
  }
  .hud-chip b { color: #60a5fa; }
  .hud-chip.amber b { color: #fbbf24; }
  .prog-track {
    position: absolute; bottom: 0; left: 0; right: 0; height: 4px; background: rgba(255, 255, 255, 0.1);
  }
  .prog-fill {
    height: 100%; width: 0%; background: var(--blue-hi); transition: width 0.18s ease-out;
  }

  /* Two-Column Telemetry & Signal Graph Grid */
  .grid-2 { display: grid; grid-template-columns: 1fr; gap: 8px; margin-bottom: 8px; }
  @media (min-width: 580px) {
    .grid-2 { grid-template-columns: 1.2fr 1fr; }
  }
  .card {
    background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; min-width: 0;
  }
  .card-title {
    font-size: 11px; font-weight: 600; color: var(--sub); text-transform: uppercase; letter-spacing: 0.4px;
    display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;
  }
  .info-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px 10px; }
  .info-item { min-width: 0; }
  .info-label { font-size: 10px; color: var(--sub); margin-bottom: 1px; }
  .info-val { font-size: 12px; font-weight: 600; color: var(--text); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  #rssiCanvas { width: 100%; height: 76px; background: #12151b; border: 1px solid var(--border); border-radius: 6px; display: block; }

  /* Action Toolbar */
  .toolbar {
    display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 6px; margin-bottom: 8px;
  }
  @media (min-width: 580px) {
    .toolbar { grid-template-columns: repeat(6, minmax(0, 1fr)); }
  }
  button, select, a.btn {
    background: var(--card-alt); color: var(--text); border: 1px solid var(--border); border-radius: 6px;
    padding: 8px 8px; font-size: 11px; font-weight: 600; cursor: pointer; text-decoration: none;
    text-align: center; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; transition: background 0.12s;
  }
  button:hover, a.btn:hover { background: #2b313e; }
  button:active, a.btn:active { transform: translateY(1px); }
  button.primary { background: var(--blue); border-color: var(--blue-hi); color: #fff; }
  button.danger { background: rgba(239, 68, 68, 0.16); border-color: rgba(239, 68, 68, 0.45); color: #fca5a5; }

  /* Capture History List */
  .log-list { max-height: 125px; overflow-y: auto; font-size: 11px; }
  .log-row {
    display: flex; justify-content: space-between; align-items: center; gap: 8px;
    padding: 6px 0; border-bottom: 1px solid var(--border); min-width: 0;
  }
  .log-row:last-child { border-bottom: none; }
  .log-row span { min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: var(--sub); }
  .log-row span b { color: var(--text); }
</style>
</head>
<body>

  <!-- Top Status Bar -->
  <div class="header">
    <div class="brand">
      <span class="dot" id="linkDot"></span>
      <span id="linkTitle">SAR Ground Link · 868 MHz</span>
    </div>
    <div class="rf-pills">
      <div class="pill">
        <span class="sig-bars" id="sigBars"><i style="height:3px"></i><i style="height:5px"></i><i style="height:7px"></i><i style="height:10px"></i><i style="height:12px"></i></span>
        <span>RSSI</span><b id="topRssi">-- dBm</b>
      </div>
      <div class="pill"><span>SNR</span><b id="topSnr">-- dB</b></div>
      <div class="pill"><span>Quality</span><b id="topQual">--%</b></div>
    </div>
  </div>

  <!-- Primary 4:3 Optical Viewport with Glass HUD Overlays -->
  <div class="viewport-wrap">
    <canvas id="viewCanvas" width="640" height="480"></canvas>
    <div class="hud-top">
      <div class="hud-chip amber" id="hudTarget">Capture #0 · <b>Standby</b></div>
      <div class="hud-chip" id="hudBlocks">Blocks: <b>0 / 300 (0%)</b></div>
    </div>
    <div class="hud-bottom">
      <div class="hud-chip" id="hudGps">GPS: <b>Waiting for telemetry...</b></div>
      <div class="hud-chip" id="hudMode">Mode: <b>Target-First Blocks</b></div>
    </div>
    <div class="prog-track"><div class="prog-fill" id="progBar"></div></div>
  </div>

  <!-- Telemetry & Signal History Cards -->
  <div class="grid-2">
    <div class="card">
      <div class="card-title">
        <span>Capture & Link Telemetry</span>
        <span id="capTimeBadge">--:--:--</span>
      </div>
      <div class="info-grid">
        <div class="info-item">
          <div class="info-label">Drone State</div>
          <div class="info-val" id="valState">Standby</div>
        </div>
        <div class="info-item">
          <div class="info-label">AI Detector</div>
          <div class="info-val" id="valModel">IMX500 SSD</div>
        </div>
        <div class="info-item">
          <div class="info-label">Unit / Sector</div>
          <div class="info-val" id="valNote">--</div>
        </div>
        <div class="info-item">
          <div class="info-label">Packets & Flash</div>
          <div class="info-val" id="valDiag">0 pkts · 0 CRC · 0 KB</div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">
        <span>Signal Power History (dBm)</span>
        <span id="ageSpan">No packets yet</span>
      </div>
      <canvas id="rssiCanvas" width="340" height="76"></canvas>
    </div>
  </div>

  <!-- Action Toolbar -->
  <div class="toolbar">
    <button class="primary" onclick="saveCanvasJpg()">Save Photo</button>
    <button onclick="saveFullReportHtml()">Export Report</button>
    <button onclick="saveRssiCsvLog()">Signal CSV</button>
    <button class="danger" onclick="clearPhoneImage()">Clear Screen</button>
    <select id="resSel" onchange="changeResolution(this.value)">
      <option value="320x240">320×240 (75 blk)</option>
      <option value="640x480" selected>640×480 (300 blk)</option>
      <option value="960x720">960×720 (675 blk)</option>
      <option value="1280x960">1280×960 (1200 blk)</option>
    </select>
    <select id="lvlSel" onchange="changeLevel(this.value)">
      <option value="0">L0 · SF12 / 125k</option>
      <option value="1">L1 · SF11 / 125k</option>
      <option value="2">L2 · SF10 / 125k</option>
      <option value="3">L3 · SF9 / 250k</option>
      <option value="4">L4 · SF8 / 250k</option>
      <option value="5" selected>L5 · SF7 / 250k</option>
      <option value="6">L6 · SF7 / 500k</option>
      <option value="7">L7 · SF6 / 500k</option>
    </select>
  </div>

  <!-- Saved Captures Log -->
  <div class="card">
    <div class="card-title">
      <span>Saved Captures</span>
      <div style="display:flex; gap:10px; align-items:center;">
        <label style="cursor:pointer; font-size:11px; color:var(--text); text-transform:none;">
          <input type="checkbox" id="autoSaveChk" checked style="vertical-align:middle;"> Auto-Save (≥90%)
        </label>
        <button onclick="clearSavedHistory()" style="padding:2px 8px; font-size:10px;">Clear Log</button>
      </div>
    </div>
    <div class="log-list" id="savedList"></div>
  </div>

<script>
const vCanvas = document.getElementById('viewCanvas');
const vCtx = vCanvas.getContext('2d');
const rCanvas = document.getElementById('rssiCanvas');
const rCtx = rCanvas.getContext('2d');

let lastCaptureId = -1;
let lastImgVer = 0;
let lastTileSeq = 0;
let lastClearSeq = -1;
let latestStatus = {};
let rssiLogRows = [];
let savedCaptures = [];
let lastAutoLoggedCaptureId = -1;
let curImgW = 640;
let curImgH = 480;
let curTileCols = 10;
let curTileRows = 30;
let drawnTiles = new Array(300).fill(false);
let drawnCount = 0;
let missingCursor = 0;
let isFetchingTiles = false;

function formatModeLabel(m) {
  if (!m) return 'Target-First Blocks';
  if (m === 'SCAN_TARGET') return 'Target-First Blocks';
  if (m === 'SCAN_ROWS') return 'Row-Order Blocks';
  if (m === '2D_FULL') return 'Full-Frame Packets';
  return m;
}

function resetDrawnTiles() {
  drawnTiles = new Array(curTileCols * curTileRows).fill(false);
  drawnCount = 0;
  missingCursor = 0;
}

function drawBlankCanvas(msg) {
  const w = vCanvas.width, h = vCanvas.height;
  vCtx.fillStyle = '#090b0e';
  vCtx.fillRect(0, 0, w, h);

  // Draw subtle 64x16 block grid so user visibly sees 5x15 (320x240), 10x30 (640x480), 15x45 (960x720), or 20x60 (1280x960)
  vCtx.strokeStyle = '#141821';
  vCtx.lineWidth = 1;
  vCtx.beginPath();
  for (let x = 64; x < w; x += 64) {
    vCtx.moveTo(x, 0); vCtx.lineTo(x, h);
  }
  for (let y = 16; y < h; y += 16) {
    vCtx.moveTo(0, y); vCtx.lineTo(w, y);
  }
  vCtx.stroke();

  vCtx.strokeStyle = '#252c3b';
  vCtx.beginPath();
  const ch = Math.max(16, Math.round(w * 0.04));
  vCtx.moveTo(w / 2, h / 2 - ch); vCtx.lineTo(w / 2, h / 2 + ch);
  vCtx.moveTo(w / 2 - ch, h / 2); vCtx.lineTo(w / 2 + ch, h / 2);
  vCtx.strokeRect(Math.round(w * 0.075), Math.round(h * 0.075), Math.round(w * 0.85), Math.round(h * 0.85));
  vCtx.stroke();

  const fSize = Math.max(11, Math.round(w * 0.022));
  vCtx.fillStyle = '#9ca3af';
  vCtx.font = `600 ${fSize}px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif`;
  vCtx.textAlign = 'center';
  vCtx.fillText(`${curImgW}×${curImgH} Viewport · ${curTileCols}×${curTileRows} Grid (${curTileCols * curTileRows} Blocks)`, w / 2, h / 2 - ch - 10);
  vCtx.fillStyle = '#6b7280';
  vCtx.fillText(msg || 'Optical Feed Standby · Waiting for Drone Transmission', w / 2, h / 2 + ch + fSize + 8);
  vCtx.textAlign = 'left';
}
drawBlankCanvas();

function applyCanvasResolution(w, h, reasonMsg) {
  w = parseInt(w, 10);
  h = parseInt(h, 10);
  if (!w || !h || w < 320 || h < 240) return;
  if (w === curImgW && h === curImgH && vCanvas.width === w && vCanvas.height === h) return;
  curImgW = w;
  curImgH = h;
  curTileCols = Math.round(w / 64);
  curTileRows = Math.round(h / 16);
  vCanvas.width = w;
  vCanvas.height = h;
  const resSel = document.getElementById('resSel');
  if (resSel) resSel.value = `${w}x${h}`;
  lastTileSeq = 0;
  resetDrawnTiles();
  drawBlankCanvas(reasonMsg || `Viewport Adjusted to ${w}×${h} (${curTileCols * curTileRows} Blocks)`);
}

async function clearPhoneImage() {
  lastTileSeq = 0;
  lastImgVer = 0;
  resetDrawnTiles();
  drawBlankCanvas('Screen Cleared · Waiting for Next Transmission');
  document.getElementById('progBar').style.width = '0%';
  document.getElementById('hudBlocks').innerHTML = `Blocks: <b>0 / ${curTileCols * curTileRows} (0%)</b>`;
  try {
    await fetch('/api/clear');
  } catch (e) {}
}

function clearSavedHistory() {
  savedCaptures = [];
  try { localStorage.removeItem('sar_saved_captures'); } catch (e) {}
  renderSavedHistoryList();
}

function updateSignalIndicators(rssi, snr, totalPkts, dropped) {
  const rssiEl = document.getElementById('topRssi');
  const snrEl = document.getElementById('topSnr');
  const qualEl = document.getElementById('topQual');
  const barsEl = document.getElementById('sigBars');
  const bars = barsEl.querySelectorAll('i');

  if (!totalPkts || totalPkts === 0) {
    rssiEl.textContent = '-- dBm';
    snrEl.textContent = '-- dB';
    qualEl.textContent = '--%';
    bars.forEach(b => b.className = '');
    return;
  }

  // Quality % from RSSI (-120 dBm -> 0%, -30 dBm -> 100%)
  const qual = Math.max(0, Math.min(100, Math.round(((rssi + 120) / 90) * 100)));
  const activeBars = dropped ? 0 : Math.max(1, Math.min(5, Math.ceil(qual / 20)));

  rssiEl.textContent = `${rssi.toFixed(0)} dBm`;
  snrEl.textContent = `${snr >= 0 ? '+' : ''}${snr.toFixed(1)} dB`;
  qualEl.textContent = `${qual}%`;

  barsEl.className = 'sig-bars' + (qual < 35 ? ' bad' : (qual < 65 ? ' warn' : ''));
  bars.forEach((b, idx) => {
    b.className = (idx < activeBars) ? 'on' : '';
  });
}

function drawRssiChart(hist, dropped) {
  const w = rCanvas.width, h = rCanvas.height;
  rCtx.fillStyle = '#12151b';
  rCtx.fillRect(0, 0, w, h);

  // Reference dBm grid lines (-30 dBm top, -75 dBm mid, -120 dBm bottom)
  rCtx.strokeStyle = '#1f2430';
  rCtx.lineWidth = 1;
  [14, h / 2, h - 12].forEach(y => {
    rCtx.beginPath();
    rCtx.moveTo(38, y);
    rCtx.lineTo(w - 6, y);
    rCtx.stroke();
  });

  rCtx.fillStyle = '#6b7280';
  rCtx.font = '10px -apple-system, sans-serif';
  rCtx.fillText('-30', 6, 17);
  rCtx.fillText('-75', 6, h / 2 + 3);
  rCtx.fillText('-120', 4, h - 9);

  if (!hist || hist.length === 0) return;

  const plotX0 = 40, plotW = w - 48;
  const strokeCol = dropped ? '#ef4444' : '#3b82f6';
  rCtx.strokeStyle = strokeCol;
  rCtx.lineWidth = 2;
  rCtx.beginPath();
  for (let i = 0; i < hist.length; i++) {
    const x = plotX0 + (i / Math.max(1, 39)) * plotW;
    const norm = Math.max(0, Math.min(1, (hist[i] + 120) / 90.0));
    const y = (h - 12) - norm * (h - 26);
    if (i === 0) rCtx.moveTo(x, y);
    else rCtx.lineTo(x, y);
  }
  rCtx.stroke();
}

function joinB64Safe(hdrB64, scanB64) {
  if (!hdrB64.endsWith('=')) return hdrB64 + scanB64;
  try {
    return btoa(atob(hdrB64) + atob(scanB64));
  } catch (e) {
    return hdrB64.replace(/=+$/, '') + scanB64;
  }
}

function drawTileBatch(hdr, tiles) {
  return new Promise(resolve => {
    if (!tiles || tiles.length === 0) {
      resolve();
      return;
    }
    let pending = tiles.length;
    const doneOne = () => {
      pending--;
      if (pending <= 0) resolve();
    };
    const totalGrid = curTileCols * curTileRows;
    for (const t of tiles) {
      const idx = t.r * curTileCols + t.c;
      const fullB64 = joinB64Safe(hdr, t.b64);
      const img = new Image();
      img.onload = () => {
        if (drawnCount === 0) {
          vCtx.fillStyle = '#090b0e';
          vCtx.fillRect(0, 0, vCanvas.width, vCanvas.height);
        }
        vCtx.drawImage(img, t.c * 64, t.r * 16, 64, 16);
        if (idx >= 0 && idx < totalGrid && !drawnTiles[idx]) {
          drawnTiles[idx] = true;
          drawnCount++;
        }
        doneOne();
      };
      img.onerror = doneOne;
      img.src = 'data:image/jpeg;base64,' + fullB64;
    }
  });
}

async function fetchPictureBlocks(sinceSeq) {
  if (isFetchingTiles) return;
  isFetchingTiles = true;
  try {
    let fromIdx = 0;
    let firstPageMaxSeq = 0;
    while (true) {
      const ctrl = new AbortController();
      const tid = setTimeout(() => ctrl.abort(), 1800);
      const r = await fetch(`/api/tiles?since=${sinceSeq}&from=${fromIdx}`, {signal: ctrl.signal});
      clearTimeout(tid);
      const d = await r.json();
      if (d.w && d.h) {
        applyCanvasResolution(d.w, d.h);
      }
      if (firstPageMaxSeq === 0 && d.maxSeq) {
        firstPageMaxSeq = d.maxSeq;
      }
      if (!d.hdr || !d.tiles || d.tiles.length === 0) {
        break;
      }
      await drawTileBatch(d.hdr, d.tiles);
      if (d.more && typeof d.nextFrom === 'number') {
        fromIdx = d.nextFrom;
      } else {
        break;
      }
    }
    if (firstPageMaxSeq > lastTileSeq) {
      lastTileSeq = firstPageMaxSeq;
    }
  } catch (e) {
  } finally {
    isFetchingTiles = false;
  }
}

async function reconcileMissingBlocks() {
  if (isFetchingTiles) return;
  const totalGrid = curTileCols * curTileRows;
  if (drawnCount >= totalGrid) return;
  const missingIds = [];
  for (let step = 0; step < totalGrid && missingIds.length < 25; step++) {
    const idx = (missingCursor + step) % totalGrid;
    if (!drawnTiles[idx]) {
      missingIds.push(idx);
    }
  }
  missingCursor = (missingCursor + 25) % Math.max(1, totalGrid);
  if (missingIds.length === 0) return;

  isFetchingTiles = true;
  try {
    const ctrl = new AbortController();
    const tid = setTimeout(() => ctrl.abort(), 1600);
    const r = await fetch(`/api/tiles?ids=${missingIds.join(',')}`, {signal: ctrl.signal});
    clearTimeout(tid);
    const d = await r.json();
    if (d.w && d.h) {
      applyCanvasResolution(d.w, d.h);
    }
    if (d.hdr && d.tiles && d.tiles.length > 0) {
      await drawTileBatch(d.hdr, d.tiles);
    }
  } catch (e) {
  } finally {
    isFetchingTiles = false;
  }
}

async function pollLoop() {
  try {
    const ctrl = new AbortController();
    const tid = setTimeout(() => ctrl.abort(), 1600);
    const r = await fetch('/api/status', {signal: ctrl.signal});
    clearTimeout(tid);
    const d = await r.json();
    latestStatus = d;

    if (d.imgW && d.imgH) {
      applyCanvasResolution(d.imgW, d.imgH);
    }

    if (lastClearSeq !== -1 && d.clearSeq !== lastClearSeq) {
      lastTileSeq = 0;
      lastImgVer = 0;
      resetDrawnTiles();
      drawBlankCanvas('Screen Cleared · Standby');
    }
    lastClearSeq = d.clearSeq;

    if (lastCaptureId !== -1 && d.captureId !== lastCaptureId) {
      lastTileSeq = 0;
      resetDrawnTiles();
    }
    lastCaptureId = d.captureId;

    document.getElementById('lvlSel').value = d.level;
    document.getElementById('ageSpan').textContent = d.totalPkts > 0
      ? `${d.rssi.toFixed(0)} dBm · SNR ${d.snr >= 0 ? '+' : ''}${d.snr.toFixed(1)} dB (${d.ageSec}s ago)`
      : 'Waiting for signal';

    const linkDot = document.getElementById('linkDot');
    const linkTitle = document.getElementById('linkTitle');
    if (d.dropped) {
      linkDot.className = 'dot offline';
      linkTitle.textContent = `Signal Standby (${d.ageSec}s)`;
    } else {
      linkDot.className = 'dot';
      linkTitle.textContent = `Drone Connected · 868 MHz`;
    }

    updateSignalIndicators(d.rssi, d.snr, d.totalPkts, d.dropped);

    const pct = d.totalChunks > 0 ? Math.round((d.receivedChunks / d.totalChunks) * 100) : 0;
    document.getElementById('progBar').style.width = pct + '%';

    const unitLbl = (d.mode === '2D_FULL') ? 'Packets' : 'Blocks';
    document.getElementById('hudBlocks').innerHTML = `${curImgW}×${curImgH} · ${unitLbl}: <b>${d.receivedChunks} / ${d.totalChunks} (${pct}%)</b>`;
    document.getElementById('hudTarget').innerHTML = d.captureId > 0
      ? `Capture #${d.captureId} · <b>Target ${d.maxConf.toFixed(0)}%</b>`
      : `Live Conf: <b>${d.liveConf.toFixed(0)}%</b>`;
    document.getElementById('hudGps').innerHTML = `GPS: <b>${d.gps || 'No Fix'}</b>`;
    document.getElementById('hudMode').innerHTML = `Mode: <b>${formatModeLabel(d.mode)}</b>`;

    document.getElementById('capTimeBadge').textContent = d.timeStr || '--:--:--';
    document.getElementById('valState').textContent = d.droneState || 'WIDE_SEARCH';
    document.getElementById('valModel').textContent = `${d.model || 'SSD'} · Q${d.jpgQuality}`;
    document.getElementById('valNote').textContent = d.note || '--';
    document.getElementById('valDiag').textContent = `${d.totalPkts} pkts · ${d.corrupted} CRC · ${d.flashKb} KB`;

    drawRssiChart(d.rssiHist, d.dropped);

    if (d.totalPkts > 0) {
      const ts = new Date().toLocaleTimeString();
      if (rssiLogRows.length === 0 || rssiLogRows[rssiLogRows.length - 1].pkt !== d.totalPkts) {
        rssiLogRows.push({
          time: ts, pkt: d.totalPkts, rssi: d.rssi, snr: d.snr,
          dropped: d.dropped ? 'DROPPED' : 'OK', state: d.droneState,
          conf: d.maxConf, gps: d.gps, mode: formatModeLabel(d.mode)
        });
        if (rssiLogRows.length > 500) rssiLogRows.shift();
      }
    }

    // 1. Stream newly arrived blocks
    if (d.tileSeq > lastTileSeq) {
      await fetchPictureBlocks(lastTileSeq);
      checkAutoLogCapture(d, pct);
    }
    // 2. Actively reconcile any missed blocks by requesting their exact grid IDs from ESP32 RAM
    if (d.mode !== '2D_FULL' && d.receivedChunks > 0 && drawnCount < (curTileCols * curTileRows)) {
      await reconcileMissingBlocks();
      checkAutoLogCapture(d, pct);
    }

    if (d.imgVer > 0 && d.imgVer !== lastImgVer) {
      lastImgVer = d.imgVer;
      const img = new Image();
      img.onload = () => {
        vCtx.drawImage(img, 0, 0, vCanvas.width, vCanvas.height);
        checkAutoLogCapture(d, pct);
      };
      img.src = '/latest.jpg?v=' + d.imgVer;
    }
  } catch (e) {}
  setTimeout(pollLoop, 500);
}

function checkAutoLogCapture(d, pct) {
  if (!document.getElementById('autoSaveChk').checked) return;
  if (d.captureId > 0 && pct >= 90 && lastAutoLoggedCaptureId !== d.captureId) {
    lastAutoLoggedCaptureId = d.captureId;
    addCaptureToLocalHistory(d);
  }
}

function addCaptureToLocalHistory(d) {
  const dataUrl = vCanvas.toDataURL('image/jpeg', 0.90);
  const rec = {
    id: d.captureId,
    time: d.timeStr || new Date().toLocaleTimeString(),
    conf: d.maxConf.toFixed(0),
    mode: formatModeLabel(d.mode),
    gps: d.gps,
    note: d.note,
    rssi: d.rssi.toFixed(0),
    snr: d.snr.toFixed(1),
    imgData: dataUrl
  };
  savedCaptures.unshift(rec);
  if (savedCaptures.length > 12) savedCaptures.pop();
  try { localStorage.setItem('sar_saved_captures', JSON.stringify(savedCaptures)); } catch (e) {}
  renderSavedHistoryList();
}

function renderSavedHistoryList() {
  const box = document.getElementById('savedList');
  if (savedCaptures.length === 0) {
    box.innerHTML = '<div style="color:#6b7280;padding:4px 0;">No captures saved yet.</div>';
    return;
  }
  box.innerHTML = savedCaptures.map((c, idx) =>
    `<div class="log-row">
      <span><b>#${c.id}</b> · ${c.time} · <b>${c.conf}%</b> · ${c.gps} · ${c.rssi} dBm</span>
      <button onclick="downloadHistoryItem(${idx})" style="padding:4px 10px;font-size:10px;">Save</button>
    </div>`
  ).join('');
}

function downloadHistoryItem(idx) {
  const c = savedCaptures[idx];
  if (!c) return;
  const a = document.createElement('a');
  a.href = c.imgData;
  a.download = `SAR_Capture_${c.id}_${c.conf}pct.jpg`;
  a.click();
}

function saveCanvasJpg() {
  const cid = latestStatus.captureId || 0;
  const conf = latestStatus.maxConf ? latestStatus.maxConf.toFixed(0) : 0;
  const a = document.createElement('a');
  a.href = vCanvas.toDataURL('image/jpeg', 0.92);
  a.download = `SAR_Capture_${cid}_${conf}pct_${curImgW}x${curImgH}.jpg`;
  a.click();
}

function saveFullReportHtml() {
  const d = latestStatus;
  const imgData = vCanvas.toDataURL('image/jpeg', 0.92);
  const rowsHtml = rssiLogRows.slice(-60).map(r =>
    `<tr><td>${r.time}</td><td>${r.pkt}</td><td>${r.rssi.toFixed(1)} dBm</td><td>${r.snr.toFixed(1)} dB</td><td>${r.dropped}</td><td>${r.conf}%</td><td>${r.gps}</td></tr>`
  ).join('');
  const html = `<!DOCTYPE html><html><head><meta charset="utf-8"><title>SAR Report #${d.captureId || 0}</title>
  <style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:#111317;color:#f3f4f6;padding:24px;}img{max-width:100%;border:1px solid #2a2f3a;border-radius:8px;}table{border-collapse:collapse;width:100%;margin-top:14px;}td,th{border:1px solid #2a2f3a;padding:7px;text-align:left;font-size:12px;}</style></head>
  <body><h2>SAR Ground Report · Capture #${d.captureId || 0} (${curImgW}×${curImgH})</h2>
  <p><b>Time:</b> ${d.timeStr || new Date().toLocaleString()} | <b>Target Confidence:</b> ${(d.maxConf||0).toFixed(1)}% | <b>Mode:</b> ${formatModeLabel(d.mode)} | <b>Detector:</b> ${d.model||''}</p>
  <p><b>GPS Coordinates:</b> ${d.gps||''} | <b>Unit:</b> ${d.note||''}</p>
  <p><b>Signal Power:</b> ${d.rssi||0} dBm / SNR ${d.snr||0} dB (Min ${d.minRssi||0} dBm, Max ${d.maxRssi||0} dBm)</p>
  <img src="${imgData}" alt="SAR Capture">
  <h3>RF Signal History</h3>
  <table><tr><th>Time</th><th>Packet</th><th>RSSI (dBm)</th><th>SNR (dB)</th><th>Link</th><th>Confidence</th><th>GPS</th></tr>${rowsHtml}</table>
  </body></html>`;
  const blob = new Blob([html], {type: 'text/html'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `SAR_Report_${d.captureId || 0}.html`;
  a.click();
}

function saveRssiCsvLog() {
  let csv = "Time,PacketNum,RSSI_dBm,SNR_dB,LinkStatus,DroneState,MaxConf_Pct,Mode,GPS\n";
  for (const r of rssiLogRows) {
    csv += `"${r.time}",${r.pkt},${r.rssi},${r.snr},${r.dropped},"${r.state}",${r.conf},"${r.mode}","${r.gps}"\n`;
  }
  const blob = new Blob([csv], {type: 'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `SAR_Signal_Log.csv`;
  a.click();
}

async function changeLevel(lvl) {
  await fetch('/api/level?set=' + lvl);
}

async function changeResolution(val) {
  const parts = val.split('x');
  if (parts.length === 2) {
    const w = parseInt(parts[0], 10);
    const h = parseInt(parts[1], 10);
    applyCanvasResolution(w, h, `Manual Viewport: ${w}×${h}`);
    try {
      await fetch(`/api/res?w=${w}&h=${h}`);
    } catch (e) {}
  }
}

try {
  const raw = localStorage.getItem('sar_saved_captures');
  if (raw) savedCaptures = JSON.parse(raw);
} catch (e) {}
renderSavedHistoryList();
pollLoop();
</script>
</body>
</html>
)rawliteral";

void serviceLoRaRx() {
  if (!rxFlag) return;
  rxFlag = false;
  // 1. Read raw 24-bit SX1262 PacketStatus BEFORE readData() clears IRQ status!
  uint32_t ps = radio.getPacketStatus();
  float instRssi = radio.getRSSI(false);
  String pkt;
  int16_t state = radio.readData(pkt);
  radio.startReceive();

  if (state == RADIOLIB_ERR_NONE && pkt.length() > 0) {
    digitalWrite(LED_PIN, HIGH);

    uint8_t rssiPktRaw = (uint8_t)((ps >> 16) & 0xFF);
    int8_t  snrPktRaw  = (int8_t)((ps >> 8) & 0xFF);
    uint8_t sigRssiRaw = (uint8_t)(ps & 0xFF);

    lastSnr = (float)snrPktRaw * 0.25f;

    // Pick valid non-zero hardware RSSI byte (RssiPkt or SignalRssiPkt)
    float calcRssi = 0.0f;
    if (rssiPktRaw > 0 && rssiPktRaw < 255) {
      calcRssi = -0.5f * (float)rssiPktRaw;
    } else if (sigRssiRaw > 0 && sigRssiRaw < 255) {
      calcRssi = -0.5f * (float)sigRssiRaw;
    } else if (instRssi < -1.0f && instRssi > -140.0f) {
      calcRssi = instRssi;
    }

    // When the Pi (+22 dBm) is close to the LoRa V4 (+20 dB external LNA), the input to SX1262
    // exceeds 0 dBm and clamps RssiPkt=0; derive true near-field dBm from the working LoRa SNR!
    if (calcRssi >= -1.0f || calcRssi <= -140.0f) {
      calcRssi = -34.0f + (lastSnr - 8.0f) * 1.25f;
      if (calcRssi > -18.0f) calcRssi = -18.0f;
      if (calcRssi < -95.0f) calcRssi = -95.0f;
    }
    lastRssi = calcRssi;

    lastPacketMs = millis();
    totalPacketsRx++;
    recordRssiSample(lastRssi);

    int start = 0;
    while (start < (int)pkt.length()) {
      int nl = pkt.indexOf('\n', start);
      if (nl < 0) nl = pkt.length();
      String line = pkt.substring(start, nl);
      processIncomingLine(line);
      start = nl + 1;
    }
    digitalWrite(LED_PIN, LOW);
  }
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  clearAllImageBuffers();

  // 1. Start Rock-Solid Wi-Fi Access Point FIRST (while internal heap has >190KB free!)
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4); // Channel 6, Max 4 clients
  WiFi.setSleep(false);                   // CRITICAL: Prevents ESP32-S3 from sleeping between beacons!
  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // Max Wi-Fi signal strength for phone connection
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("\r\n=== PHONE WI-FI HOTSPOT READY (NO-SLEEP MODE) ===\r\nSSID: %s\r\nPASS: %s\r\nPhone URL: http://%s | Free Heap: %u B\r\n",
                AP_SSID, AP_PASS, ip.toString().c_str(), ESP.getFreeHeap());

  // 2. Mount Onboard Flash FileSystem (LittleFS) to cache tiles, metadata & RSSI logs!
  if (LittleFS.begin(true)) {
    hasFlashFS = true;
    Serial.printf("[FLASH] LittleFS Mounted! Total: %u KB, Used: %u KB\r\n",
                  LittleFS.totalBytes() / 1024, LittleFS.usedBytes() / 1024);
    restoreFromFlashOnBoot();
  } else {
    Serial.println("[FLASH] LittleFS Mount Failed (running in RAM-only mode)");
  }

  // 3. Configure LoRa V4 Front-End Module for High-Sensitivity Receive (LNA active, PA off)
  pinMode(FEM_VFEM, OUTPUT);
  pinMode(FEM_CSD, OUTPUT);
  pinMode(FEM_CPS_V42, OUTPUT);
  pinMode(FEM_CPS_V43, OUTPUT);
  digitalWrite(FEM_VFEM, HIGH);
  digitalWrite(FEM_CSD, HIGH);
  digitalWrite(FEM_CPS_V42, LOW);
  digitalWrite(FEM_CPS_V43, LOW);
  delay(20);

  // 4. Configure Web Server Routes
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/status", HTTP_GET, []() {
    serviceLoRaRx();
    rebuildProgressiveJpegIfIdle();
    unsigned long ageSec = (lastPacketMs > 0) ? ((millis() - lastPacketMs) / 1000UL) : 999UL;
    bool dropped = (lastPacketMs == 0) || (ageSec >= 7);
    uint32_t flashKb = hasFlashFS ? (LittleFS.usedBytes() / 1024) : 0;

    String histJson = "[";
    for (int i = 0; i < rssiHistCount; i++) {
      if (i > 0) histJson += ",";
      histJson += String(rssiHistory[i]);
    }
    histJson += "]";

    String json = "{";
    json += "\"captureId\":" + String(detCaptureId) + ",";
    json += "\"personCount\":" + String(detPersonCount) + ",";
    json += "\"maxConf\":" + String(detMaxConf, 1) + ",";
    json += "\"liveConf\":" + String(droneLiveConf, 1) + ",";
    json += "\"hbSeq\":" + String(heartbeatSeq) + ",";
    json += "\"droneState\":\"" + droneZoomState + "\",";
    json += "\"mode\":\"" + detModeStr + "\",";
    json += "\"model\":\"" + detModelStr + "\",";
    json += "\"timeStr\":\"" + detTimeStr + "\",";
    json += "\"gps\":\"" + detGpsStr + "\",";
    json += "\"note\":\"" + detNoteStr + "\",";
    json += "\"jpgQuality\":" + String(detJpgQuality) + ",";
    json += "\"imgW\":" + String(imgWidth) + ",";
    json += "\"imgH\":" + String(imgHeight) + ",";
    json += "\"tileCols\":" + String(tileCols) + ",";
    json += "\"tileRows\":" + String(tileRows) + ",";
    json += "\"receivedChunks\":" + String(receivedChunks) + ",";
    json += "\"totalChunks\":" + String(totalChunks) + ",";
    json += "\"corrupted\":" + String(corruptedPackets) + ",";
    json += "\"rssi\":" + String(lastRssi, 1) + ",";
    json += "\"snr\":" + String(lastSnr, 1) + ",";
    json += "\"minRssi\":" + String(minRssi, 1) + ",";
    json += "\"maxRssi\":" + String(maxRssi, 1) + ",";
    json += "\"totalPkts\":" + String(totalPacketsRx) + ",";
    json += "\"drops\":" + String(linkDropCount) + ",";
    json += "\"dropped\":" + String(dropped ? "true" : "false") + ",";
    json += "\"flashKb\":" + String(flashKb) + ",";
    json += "\"rssiHist\":" + histJson + ",";
    json += "\"imgVer\":" + String(imageVersion) + ",";
    json += "\"tileSeq\":" + String(globalTileSeq) + ",";
    json += "\"clearSeq\":" + String(clearSeq) + ",";
    json += "\"level\":" + String(currentLevel) + ",";
    json += "\"levelName\":\"" + String(LEVELS[currentLevel].name) + "\",";
    json += "\"ageSec\":" + String(ageSec);
    json += "}";
    server.send(200, "application/json", json);
    serviceLoRaRx();
  });

  server.on("/api/clear", HTTP_GET, []() {
    clearImageAndFlash();
    server.send(200, "text/plain", "CLEARED");
  });

  server.on("/api/tiles", HTTP_GET, []() {
    serviceLoRaRx();
    String json;
    json.reserve(4096);
    json = "{\"w\":" + String(imgWidth) + ",\"h\":" + String(imgHeight) + ",\"hdr\":\"" + commonTileHeaderB64 + "\",\"maxSeq\":" + String(globalTileSeq) + ",\"tiles\":[";
    bool first = true;
    int sentCount = 0;
    bool hasMore = false;
    int nextFrom = activeNumTiles;

    if (server.hasArg("ids")) {
      // Direct missing-block reconciliation by exact grid indices requested by the phone
      String idsArg = server.arg("ids");
      int sPos = 0;
      while (sPos < (int)idsArg.length() && sentCount < 25) {
        int comma = idsArg.indexOf(',', sPos);
        if (comma < 0) comma = idsArg.length();
        int t = idsArg.substring(sPos, comma).toInt();
        sPos = comma + 1;
        if (t >= 0 && t < activeNumTiles && t < MAX_TILES) {
          const char* slot = getSlot(t);
          if (slot != nullptr && slot[0] != '\0') {
            if (!first) json += ",";
            first = false;
            int r = t / tileCols;
            int c = t % tileCols;
            json += "{\"c\":" + String(c) + ",\"r\":" + String(r) + ",\"seq\":" + String(tileSeq[t]) + ",\"b64\":\"" + String(slot) + "\"}";
            sentCount++;
          }
        }
      }
    } else {
      uint32_t sinceSeq = 0;
      if (server.hasArg("since")) {
        sinceSeq = (uint32_t)server.arg("since").toInt();
      }
      int fromIdx = 0;
      if (server.hasArg("from")) {
        fromIdx = server.arg("from").toInt();
        if (fromIdx < 0) fromIdx = 0;
      }
      for (int t = fromIdx; t < activeNumTiles && t < MAX_TILES; t++) {
        const char* slot = getSlot(t);
        if (tileSeq[t] > sinceSeq && slot != nullptr && slot[0] != '\0') {
          if (sentCount >= 20) {
            hasMore = true;
            nextFrom = t;
            break;
          }
          if (!first) json += ",";
          first = false;
          int r = t / tileCols;
          int c = t % tileCols;
          json += "{\"c\":" + String(c) + ",\"r\":" + String(r) + ",\"seq\":" + String(tileSeq[t]) + ",\"b64\":\"" + String(slot) + "\"}";
          sentCount++;
        }
      }
    }
    json += "],\"more\":" + String(hasMore ? "true" : "false") + ",\"nextFrom\":" + String(nextFrom) + "}";
    server.send(200, "application/json", json);
    serviceLoRaRx();
  });

  server.on("/api/flash_rssi.csv", HTTP_GET, []() {
    if (!hasFlashFS || !LittleFS.exists("/rssi.csv")) {
      server.send(200, "text/csv", "UptimeSec,PacketNum,RSSI_dBm,SNR_dB,Event,LiveConf,GPS\n");
      return;
    }
    File f = LittleFS.open("/rssi.csv", "r");
    server.streamFile(f, "text/csv");
    f.close();
  });

  server.on("/api/level", HTTP_GET, []() {
    if (server.hasArg("set")) {
      int lvl = server.arg("set").toInt();
      applyLoRaLevel(lvl);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/res", HTTP_GET, []() {
    if (server.hasArg("w") && server.hasArg("h")) {
      setResolutionGrid(server.arg("w").toInt(), server.arg("h").toInt());
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/latest.jpg", HTTP_GET, []() {
    rebuildProgressiveJpegIfIdle();
    if (latestJpgLen == 0 || latestJpgBuf == nullptr) {
      server.send(404, "text/plain", "No image received yet");
      return;
    }
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    WiFiClient client = server.client();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(latestJpgLen));
    client.println("Connection: close");
    client.println();
    client.write(latestJpgBuf, latestJpgLen);
  });

  server.begin();

  // 4. Initialize Internal SX1262 SPI Radio at 868.0 MHz using RadioLib ConfigLoRa_t
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  ConfigLoRa_t loraCfg;
  loraCfg.frequency       = RADIOLIB_UNIT_MEGA(868);
  loraCfg.bandwidth       = RADIOLIB_UNIT_KILO(250);
  loraCfg.spreadingFactor = 7;
  loraCfg.codingRate      = 5;
  loraCfg.syncWord        = 0x12; // Matches 0x1424 on Raspberry Pi Waveshare SX1262 HAT
  loraCfg.power           = 22;
  loraCfg.preambleLength  = 8;

  radio.tcxoVoltage = Voltage1V8;
  radio.useRegulatorLDO = false;
  int16_t state = radio.begin(loraCfg);
  if (state != RADIOLIB_ERR_NONE) {
    radio.tcxoVoltage = VoltageNone;
    state = radio.begin(loraCfg);
  }

  radio.setDio2AsRfSwitch(true);
  radio.setCRC(2);
  radio.invertIQ(false);
  radio.setRxBoostedGainMode(true);
  radio.setDio1Action(onLoRaRx);
  applyLoRaLevel(currentLevel);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("SX1262_V4_PHONE_RECEIVER_READY @ 868.0MHz");
  } else {
    Serial.printf("SX1262_INIT_ERR:%d\r\n", state);
  }
}

void loop() {
  serviceLoRaRx();
  server.handleClient();
  serviceLoRaRx();
  rebuildProgressiveJpegIfIdle();
  flushToFlashIfIdle();

  if (lastPacketMs > 0) {
    bool nowDropped = ((millis() - lastPacketMs) >= 7000UL);
    if (nowDropped && !wasDropped) {
      wasDropped = true;
      linkDropCount++;
      appendRssiToFlash(lastRssi, lastSnr, "DROP");
    } else if (!nowDropped && wasDropped) {
      wasDropped = false;
    }
  }
}
