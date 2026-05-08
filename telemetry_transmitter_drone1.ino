#include <RadioLib.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <INA226_WE.h>

// ================== DRONE ID ==================
#define DRONE_ID "DRONE1"

// ================== LIPO CELL COUNT ==================
//   1S = 1  (3.7V single-cell test battery)
//   3S = 3  (11.1V nominal)
//   4S = 4  (14.8V nominal)
#define LIPO_CELLS 3

// ================== LoRa pins (SX1262) ==================
#define LORA_NSS_PIN   8
#define LORA_DIO1_PIN  14
#define LORA_RST_PIN   12
#define LORA_BUSY_PIN  13

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

// ================== GPS pins (MAX-M10S) ==================
#define GPS_RX_PIN  44
#define GPS_TX_PIN  43

// Target baud rate for GPS after configuration.
// MAX-M10S supports 9600 / 38400 / 115200.

#define GPS_BAUD_TARGET 9600

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// ================== BME280 ==================
Adafruit_BME280 bme;
bool bme_ok = false;
#define SEALEVEL_HPA 1013.25f

bool          takeoff_set    = false;
float         takeoff_balt_m = 0.0f;
unsigned long last_balt_ms   = 0;
float         last_balt_m    = 0.0f;
float         vspd_mps       = 0.0f;

// ================== INA226 ==================
#define INA226_ADDR          0x40
#define SHUNT_RESISTANCE_OHM 0.1f
#define MAX_EXPECTED_CURRENT_A 1.0f

INA226_WE ina226(INA226_ADDR);
bool ina_ok = false;

// ================== LiPo voltage → SOC table ==================
struct VoltageSOC { float volts; uint8_t pct; };
static const VoltageSOC LIPO_CURVE[] = {
  { 4.20f, 100 }, { 4.05f, 90 }, { 3.90f, 75 }, { 3.80f, 60 },
  { 3.70f,  40 }, { 3.60f, 20 }, { 3.50f, 10 }, { 3.40f,  5 }, { 3.00f, 0 },
};
static const uint8_t CURVE_LEN = sizeof(LIPO_CURVE) / sizeof(LIPO_CURVE[0]);

float voltageToPercent(float packV) {
  float v = packV / (float)LIPO_CELLS;
  if (v >= LIPO_CURVE[0].volts)             return 100.0f;
  if (v <= LIPO_CURVE[CURVE_LEN-1].volts)   return   0.0f;
  for (uint8_t i = 0; i < CURVE_LEN - 1; i++) {
    if (v <= LIPO_CURVE[i].volts && v > LIPO_CURVE[i+1].volts) {
      float t = (v - LIPO_CURVE[i+1].volts) / (LIPO_CURVE[i].volts - LIPO_CURVE[i+1].volts);
      return LIPO_CURVE[i+1].pct + t * (LIPO_CURVE[i].pct - LIPO_CURVE[i+1].pct);
    }
  }
  return 0.0f;
}

#define BATT_SMOOTH_N 8
float   batt_buf[BATT_SMOOTH_N] = {};
uint8_t batt_idx  = 0;
bool    batt_full = false;

float smoothedBattVoltage(float s) {
  batt_buf[batt_idx] = s;
  batt_idx = (batt_idx + 1) % BATT_SMOOTH_N;
  if (batt_idx == 0) batt_full = true;
  uint8_t n = batt_full ? BATT_SMOOTH_N : batt_idx;
  float sum = 0; for (uint8_t i = 0; i < n; i++) sum += batt_buf[i];
  return sum / n;
}

// ================== GPS helpers ==================
void readGpsStream() {
  while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
}

// --- UBX helper: send a UBX message and wait for ACK ---
void sendUBX(const uint8_t *msg, size_t len) {
  GPSSerial.write(msg, len);
  GPSSerial.flush();
  delay(200);   // increased: give module time to process each command
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
    // scanChannel() runs a proper CAD cycle on the SX1262 hardware.
    // Returns RADIOLIB_CHANNEL_FREE or RADIOLIB_PREAMBLE_DETECTED.
    // This blocks for ~1-2 LoRa symbols (~0.5 ms at SF7/BW250).
    int16_t cad = radio.scanChannel();

    if (cad == RADIOLIB_CHANNEL_FREE) {
      break;   // clear — transmit now
    }

    // Channel busy — wait a random backoff before retrying.
    // random() gives different sequences on each drone naturally since
    // millis() will differ slightly at the point of first call.
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

  Wire.begin(4, 5);

  // BME280
  bme_ok = bme.begin(0x76) || bme.begin(0x77);
  Serial.println(bme_ok ? F("BME280 OK") : F("BME280 NOT FOUND"));

  // INA226
  ina_ok = ina226.init();
  if (ina_ok) {
    ina226.setResistorRange(SHUNT_RESISTANCE_OHM, MAX_EXPECTED_CURRENT_A);
    ina226.setAverage(INA226_AVERAGE_16);
    ina226.setConversionTime(INA226_CONV_TIME_1100);
    ina226.setMeasureMode(INA226_CONTINUOUS);
    Serial.println(F("INA226 OK"));
  } else {
    Serial.println(F("INA226 NOT FOUND — check address/wiring"));
  }

  // GPS: start at 9600 (skipping baud change for reliability)
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1500);  // wait for GPS to fully boot before sending UBX commands
  configureGps5Hz();
  Serial.println(F("GPS rate set to 5 Hz."));
  configureGpsNmeaSentences();
  Serial.println(F("GPS NMEA sentences filtered (GGA+RMC only)."));

  // LoRa
  Serial.print(F("Initializing LoRa... "));
  int16_t state = radio.begin(868.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("failed, code ")); Serial.println(state);
    while (true) { delay(1000); }
  }
  // SF7 + BW250 = fast enough airtime for 2 drones at 5 Hz, ~500m range.
  // Increase SF / decrease BW to trade speed for range if needed.
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
  // CSMA inside sendLoRaPacket() handles collision avoidance — no slot logic needed.
  if (!( (gps_valid && gps_updated) || (!gps_valid && timer_fired) )) return;

  last_tx_ms = now_ms;

  // ---- GPS fields ----
  bool   has_gps  = gps_valid;
  double lat      = has_gps ? gps.location.lat()                           : 0.0;
  double lon      = has_gps ? gps.location.lng()                           : 0.0;
  double alt      = (has_gps && gps.altitude.isValid())  ? gps.altitude.meters()  : 0.0;
  double hdop     = (has_gps && gps.hdop.isValid())      ? gps.hdop.hdop()        : 0.0;
  int    sats     = gps.satellites.isValid()              ? gps.satellites.value() : 0;
  double speedKmh = (has_gps && gps.speed.isValid())     ? gps.speed.kmph()       : 0.0;

  // ---- Build payload ----
  String payload;
  payload.reserve(256);

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

  // ---- BME280 ----
  if (bme_ok) {
    float tempC    = bme.readTemperature();
    float hum      = bme.readHumidity();
    float pres_hPa = bme.readPressure() / 100.0f;
    float balt_m   = bme.readAltitude(SEALEVEL_HPA);

    if (!takeoff_set) {
      bool gps_ready    = has_gps && sats >= 4 && hdop > 0.0 && hdop <= 2.5;
      // indoor_ready: allow ~3 s for the BME280 pressure reading to stabilise
      // before locking the takeoff reference (avoids a biased RALT=0 baseline).
      bool indoor_ready = !has_gps && (now_ms > 3000);
      if (gps_ready || indoor_ready) {
        takeoff_balt_m = balt_m;
        takeoff_set    = true;
        last_balt_m    = balt_m;
        last_balt_ms   = now_ms;
        Serial.println(indoor_ready ? F("Takeoff ref set (indoor).") : F("Takeoff ref set (GPS)."));
      }
    }

    if (takeoff_set) {
      unsigned long dt_ms = now_ms - last_balt_ms;
      if (dt_ms >= 200) {
        vspd_mps     = (balt_m - last_balt_m) / (dt_ms / 1000.0f);
        last_balt_m  = balt_m;
        last_balt_ms = now_ms;
      }
    }

    payload += F(",TEMP="); payload += String(tempC, 1);
    payload += F(",HUM=");  payload += String(hum, 1);
    payload += F(",PRES="); payload += String(pres_hPa, 1);
    // BALT is always present when BME is online — receiver must not gate on FIX.
    payload += F(",BALT="); payload += String(balt_m, 1);

    if (takeoff_set) {
      payload += F(",RALT="); payload += String(balt_m - takeoff_balt_m, 1);
      payload += F(",VSPD="); payload += String(vspd_mps, 2);
    } else {
      payload += F(",RALT=NA,VSPD=NA");
    }
  } else {
    // BME280 not found — emit NA for every baro field so the receiver
    // always sees a consistent set of keys regardless of hardware state.
    payload += F(",TEMP=NA,HUM=NA,PRES=NA,BALT=NA,RALT=NA,VSPD=NA");
  }

  // ---- INA226 ----
  if (ina_ok) {
    float rawV   = ina226.getBusVoltage_V();
    float ampA   = ina226.getCurrent_A();
    float wattW  = ina226.getBusPower();
    float smoothV = smoothedBattVoltage(rawV);
    float pct     = voltageToPercent(smoothV);

    payload += F(",BVOLT="); payload += String(smoothV, 2);
    payload += F(",BAMP=");  payload += String(ampA, 2);
    payload += F(",BWATT="); payload += String(wattW, 1);
    payload += F(",BATT=");  payload += String(pct, 1);

    if (pct < 15.0f) {
      Serial.print(F("⚠ LOW BATTERY: ")); Serial.print(smoothV, 2);
      Serial.print(F("V (")); Serial.print(pct, 0); Serial.println(F("%)"));
    }
  } else {
    payload += F(",BVOLT=NA,BAMP=NA,BWATT=NA,BATT=NA");
  }

  Serial.print(F("TX: ")); Serial.println(payload);
  sendLoRaPacket(payload);
}
