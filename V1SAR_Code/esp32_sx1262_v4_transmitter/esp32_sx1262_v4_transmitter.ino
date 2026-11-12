#include <SPI.h>
#include <RadioLib.h>

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
#define FEM_CPS_V42 46  // PA TX mode enable (V4.2)
#define FEM_CPS_V43 5   // PA TX mode enable (V4.3)
#define LED_PIN     35  // Onboard white status LED

SPIClass loraSPI(HSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

struct LevelConfig {
  uint8_t sf;
  uint32_t bwHz;
  uint8_t cr;
  bool ldro;
};

// 8 Speed Levels matching Raspberry Pi Waveshare SX1262 SPI HAT
const LevelConfig LEVELS[8] = {
  {12, 125000UL, 5, true},   // LEVEL 0: SF12, 125kHz (Extreme Range)
  {11, 125000UL, 5, true},   // LEVEL 1: SF11, 125kHz (Very Long Range)
  {10, 125000UL, 5, false},  // LEVEL 2: SF10, 125kHz (Long Range)
  {9,  250000UL, 5, false},  // LEVEL 3: SF9,  250kHz (Medium Range)
  {8,  250000UL, 5, false},  // LEVEL 4: SF8,  250kHz (Medium-Fast)
  {7,  250000UL, 5, false},  // LEVEL 5: SF7,  250kHz (Fast SPI LoRa - Default)
  {7,  500000UL, 5, false},  // LEVEL 6: SF7,  500kHz (Very Fast SPI LoRa)
  {6,  500000UL, 5, false}   // LEVEL 7: SF6,  500kHz (Max Turbo SPI LoRa)
};

int currentLevel = 5;
String serialBuf = "";

void applyLoRaLevel(int lvl) {
  if (lvl < 0 || lvl > 7) return;
  currentLevel = lvl;
  radio.standby();
  radio.setSpreadingFactor(LEVELS[lvl].sf);
  radio.setBandwidth(LEVELS[lvl].bwHz);
  radio.setCodingRate(LEVELS[lvl].cr);
  radio.forceLDRO(LEVELS[lvl].ldro);
  Serial.printf("+LEVEL=%d (SF%d BW%lukHz) OK\r\n", lvl, LEVELS[lvl].sf, LEVELS[lvl].bwHz / 1000UL);
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  delay(300);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 1. Enable the LoRa V4 onboard Power Amplifier / Front-End Module (FEM)
  pinMode(FEM_VFEM, OUTPUT);
  pinMode(FEM_CSD, OUTPUT);
  pinMode(FEM_CPS_V42, OUTPUT);
  pinMode(FEM_CPS_V43, OUTPUT);
  digitalWrite(FEM_VFEM, HIGH);
  digitalWrite(FEM_CSD, HIGH);
  digitalWrite(FEM_CPS_V42, HIGH);
  digitalWrite(FEM_CPS_V43, HIGH);
  delay(20);

  // 2. Start SPI & Initialize SX1262 at 868.0 MHz using RadioLib ConfigLoRa_t
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  ConfigLoRa_t loraCfg;
  loraCfg.frequency       = RADIOLIB_UNIT_MEGA(868);  // 868.0 MHz (868000000 Hz)
  loraCfg.bandwidth       = RADIOLIB_UNIT_KILO(250);  // 250.0 kHz (250000 Hz)
  loraCfg.spreadingFactor = 7;                        // SF7
  loraCfg.codingRate      = 5;                        // CR 4/5
  loraCfg.syncWord        = 0x12;                     // Private network (0x1424 on Pi)
  loraCfg.power           = 22;                       // 22 dBm SX1262 output into V4 PA
  loraCfg.preambleLength  = 8;                        // 8 symbols

  // Try 1.8V TCXO first (standard on Heltec/diymore V3/V4), fallback to XTAL if needed
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
  radio.setCurrentLimit(140.0);
  applyLoRaLevel(currentLevel);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("SX1262_V4_READY @ 868.0MHz");
  } else {
    Serial.printf("SX1262_INIT_ERR:%d\r\n", state);
  }
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 0) {
        if (serialBuf == "+++" || serialBuf == "AT" || serialBuf == "AT+RESET") {
          Serial.println("OK");
        }
        else if (serialBuf.startsWith("AT+LEVEL")) {
          int lvl = serialBuf.substring(8).toInt();
          applyLoRaLevel(lvl);
        }
        else {
          // Transmit line over SPI SX1262 LoRa!
          digitalWrite(LED_PIN, HIGH);
          radio.transmit(serialBuf);
          digitalWrite(LED_PIN, LOW);
        }
      }
      serialBuf = "";
    }
    else if (c != '\r') {
      serialBuf += c;
    }
  }
}
