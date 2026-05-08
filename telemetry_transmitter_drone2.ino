#include <RadioLib.h>
#include <TinyGPS++.h>

// ================== DRONE ID ==================
#define DRONE_ID "DRONE2"

// ================== LoRa pins (SX1262) ==================
#define LORA_NSS_PIN   41
#define LORA_DIO1_PIN  39
#define LORA_RST_PIN   42
#define LORA_BUSY_PIN  40

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

// ================== GPS pins (MAX-M10S) ==================
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43

// Staying at 9600 baud for reliability — no baud-rate change sent.
// At 9600 with only GGA+RMC enabled, 5 Hz is still achievable.
#define GPS_BAUD_TARGET 9600

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// ================== GPS helpers ==================
void readGpsStream() {
  while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
}

// --- UBX helper: send a UBX message and wait for ACK ---
void sendUBX(const uint8_t *msg, size_t len) {
  GPSSerial.write(msg, len);
  GPSSerial.flush();
  delay(200);   // give module time to process each command
}

// --- UBX-CFG-RATE: 5 Hz (200 ms measurement interval) ---
void configureGps5Hz() {
  uint8_t msg[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00,   // measRate = 200 ms
    0x01, 0x00,   // navRate  = 1
    0x01, 0x00,   // timeRef  = GPS
    0xDE, 0x6A    // checksum
  };
  sendUBX(msg, sizeof(msg));
}

// --- UBX-CFG-MSG: enable or disable an NMEA sentence ---
// msgClass=0xF0 for standard NMEA.  rate=0 disables, rate=1 enables.
void setNmeaRate(uint8_t msgId, uint8_t rate) {
  uint8_t msg[11] = {
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,
    0xF0, msgId, rate,
    0x00, 0x00   // checksum placeholder
  };
  uint8_t ck_a = 0, ck_b = 0;
  for (uint8_t i = 2; i < 9; i++) { ck_a += msg[i]; ck_b += ck_a; }
  msg[9]  = ck_a;
  msg[10] = ck_b;
  sendUBX(msg, 11);
}

void configureGpsNmeaSentences() {
  // Send each command twice — some modules need a repeat to accept it.
  // Keep only GGA and RMC; disable everything else.
  for (uint8_t pass = 0; pass < 2; pass++) {
    setNmeaRate(0x00, 1);   // GGA  ON
    setNmeaRate(0x04, 1);   // RMC  ON
    setNmeaRate(0x01, 0);   // GLL  OFF
    setNmeaRate(0x02, 0);   // GSA  OFF
    setNmeaRate(0x03, 0);   // GSV  OFF
    setNmeaRate(0x05, 0);   // VTG  OFF
  }
}

// ================== CSMA transmit ==================
// Uses RadioLib's built-in Channel Activity Detection (CAD) to check if
// another drone is transmitting before sending. If busy, waits a random
// backoff and retries. Scales to any number of drones with no configuration.
//
// CSMA_MAX_RETRIES: give up and transmit anyway after this many busy detections
//                   (prevents a drone going permanently silent in a noisy env)
// CSMA_SLOT_MS:     base backoff unit in ms — random(1,6) * CSMA_SLOT_MS per retry
#define CSMA_MAX_RETRIES  8
#define CSMA_SLOT_MS     15

bool sendLoRaPacket(String &payload) {
  for (uint8_t attempt = 0; attempt < CSMA_MAX_RETRIES; attempt++) {
    int16_t cad = radio.scanChannel();
    if (cad == RADIOLIB_CHANNEL_FREE) break;
    uint16_t backoff = random(1, 6) * CSMA_SLOT_MS;
    Serial.print(F("CSMA busy, backoff "));
    Serial.print(backoff);
    Serial.println(F(" ms"));
    delay(backoff);
  }
  int16_t state = radio.transmit(payload);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("LoRa TX failed, code ")); Serial.println(state);
    return false;
  }
  return true;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("DRONE2 transmitter starting..."));

  // GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1500);  // wait for GPS to fully boot before sending UBX commands
  configureGps5Hz();
  Serial.println(F("GPS rate set to 5 Hz."));
  configureGpsNmeaSentences();
  Serial.println(F("GPS NMEA sentences filtered (GGA+RMC only)."));

  // LoRa — settings MUST match DRONE1 and the receiver exactly
  Serial.print(F("Initializing LoRa... "));
  int16_t state = radio.begin(868.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code ")); Serial.println(state);
    while (true) { delay(1000); }
  }
  // SF7 + BW250 = fast enough airtime for 2 drones at 5 Hz, ~500m range.
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  radio.setCodingRate(5);
  radio.setOutputPower(14);
  Serial.println(F("LoRa OK (SF7, BW250)."));

  Serial.println(F("Waiting for GPS fix..."));
}

// ================== Transmit timing ==================
#define NO_GPS_TX_INTERVAL_MS 500   // fallback rate when no GPS fix (indoors)
unsigned long last_tx_ms = 0;

// ================== LOOP ==================
void loop() {
  readGpsStream();

  unsigned long now_ms = millis();

  bool gps_valid   = gps.location.isValid();
  bool gps_updated = gps.location.isUpdated();
  bool timer_fired = (now_ms - last_tx_ms) >= NO_GPS_TX_INTERVAL_MS;

  // Transmit when GPS has a fresh fix, or on the fallback timer (no fix).
  // CSMA inside sendLoRaPacket() handles collision avoidance.
  if (!( (gps_valid && gps_updated) || (!gps_valid && timer_fired) )) return;

  last_tx_ms = now_ms;

  // ---- GPS fields ----
  bool   has_gps  = gps_valid;
  double lat      = has_gps ? gps.location.lat()                          : 0.0;
  double lon      = has_gps ? gps.location.lng()                          : 0.0;
  double alt      = (has_gps && gps.altitude.isValid())  ? gps.altitude.meters()  : 0.0;
  double hdop     = (has_gps && gps.hdop.isValid())      ? gps.hdop.hdop()        : 0.0;
  int    sats     = gps.satellites.isValid()              ? gps.satellites.value() : 0;
  double speedKmh = (has_gps && gps.speed.isValid())     ? gps.speed.kmph()       : 0.0;

  // ---- Build payload ----
  // GPS-only drone: no BME280/INA226, so payload is smaller than DRONE1.
  String payload;
  payload.reserve(128);

  payload  = F("ID=");     payload += DRONE_ID;
  payload += F(",FIX=");   payload += (has_gps ? F("1") : F("0"));

  if (has_gps) {
    payload += F(",LAT=");  payload += String(lat, 6);
    payload += F(",LON=");  payload += String(lon, 6);
    payload += F(",ALT=");  payload += String(alt, 1);
    payload += F(",HDOP="); payload += String(hdop, 1);
    payload += F(",SATS="); payload += String(sats);
    payload += F(",SPEED=");payload += String(speedKmh, 1);
  } else {
    payload += F(",LAT=NA,LON=NA,ALT=NA,HDOP=NA");
    payload += F(",SATS="); payload += String(sats);
    payload += F(",SPEED=NA");
  }

  Serial.print(F("TX: ")); Serial.println(payload);
  sendLoRaPacket(payload);
}
