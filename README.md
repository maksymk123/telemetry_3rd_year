# Multi-Drone LoRa Telemetry System

A real-time telemetry system for multi-drone monitoring over LoRa, with a Python ground station GUI. Developed as a 3rd Year Individual Project at the University of Manchester, Department of Electrical and Electronic Engineering (EEEN30330), 2025/26.

The system transmits GPS position, barometric altitude, relative altitude, vertical speed, environmental data, and battery telemetry from up to two drones simultaneously to a single receiver. A Python desktop application displays live data, plots flight paths on an interactive map, and logs all received packets.

---

## Repository Structure

```
├── telemetry_transmitter_drone1.ino   # Drone 1 firmware (GPS + BME280 + INA226)
├── telemetry_transmitter_drone2.ino   # Drone 2 firmware (GPS only)
├── telemetry_receiver.ino             # Receiver firmware (LoRa → USB serial)
├── main.py                            # Python ground station GUI
└── hardware/                          # Altium Designer project for Drone 1 PCB
```

---

## Hardware

### Drone 1 (Full telemetry — GPS, barometer, power monitor)

| Component | Part |
|---|---|
| Microcontroller + LoRa | Heltec ESP32 LoRa V3 (ESP32-S3, SX1262) |
| GPS | Uputronics u-blox MAX-M10S Breakout for Active Antennas (3.3V) |
| Barometer / Temp / Humidity | Bosch BME280 |
| Power monitor | INA226 (shunt: 0.1 Ω) |
| Battery | 3S LiPo, 11.1V nominal, 2200 mAh |

### Drone 2 (GPS telemetry only — test / second drone)

| Component | Part |
|---|---|
| Microcontroller | Seeed XIAO ESP32-S3 |
| LoRa | Seeed Wio-SX1262 (paired with XIAO) |
| GPS | SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic) |

### Receiver (Ground station)

| Component | Part |
|---|---|
| Microcontroller + LoRa | Heltec ESP32 LoRa V3 (ESP32-S3, SX1262) |

---

## LoRa Configuration

All three devices **must** share the same radio settings or they will not communicate.

| Parameter | Value |
|---|---|
| Frequency | 868.0 MHz (EU ISM band) |
| Spreading Factor | SF7 |
| Bandwidth | 250 kHz |
| Coding Rate | 4/5 |
| Output Power | 14 dBm |

Collision avoidance is handled by CSMA with RadioLib's built-in Channel Activity Detection (CAD). Each transmitter checks the channel before sending and backs off with a random delay if busy. This allows multiple drones to share the channel with no central coordination.

---

## Packet Format

Packets are ASCII strings of comma-separated `KEY=VALUE` pairs, terminated by a newline. The receiver forwards them verbatim over USB serial to the Python GUI.

### Drone 1 example (GPS fix acquired)
```
ID=DRONE1,FIX=1,LAT=53.468900,LON=-2.233500,ALT=82.4,HDOP=1.2,SATS=9,SPEED=3.1,TEMP=18.4,HUM=62.3,PRES=1008.7,BALT=142.6,RALT=0.0,VSPD=0.00,BATT=11.43,CURR=0.312,PWR=3568
```

### Drone 1 example (no GPS fix)
```
ID=DRONE1,FIX=0,LAT=NA,LON=NA,ALT=NA,HDOP=NA,SATS=0,SPEED=NA,TEMP=18.4,HUM=62.3,PRES=1008.7,BALT=142.6,RALT=0.0,VSPD=0.00,BATT=11.43,CURR=0.312,PWR=3568
```

### Key reference

| Key | Unit | Source | Notes |
|---|---|---|---|
| `ID` | — | firmware | Drone identifier string |
| `FIX` | 0/1 | GPS | 1 = valid GPS fix |
| `LAT` / `LON` | degrees | GPS | 6 decimal places; `NA` if no fix |
| `ALT` | m | GPS | Altitude above sea level (GPS); `NA` if no fix |
| `HDOP` | — | GPS | Horizontal dilution of precision; `NA` if no fix |
| `SATS` | count | GPS | Satellites in view (available without fix) |
| `SPEED` | km/h | GPS | Ground speed; `NA` if no fix |
| `TEMP` | °C | BME280 | Drone 1 only |
| `HUM` | % | BME280 | Relative humidity; Drone 1 only |
| `PRES` | hPa | BME280 | Atmospheric pressure; Drone 1 only |
| `BALT` | m | BME280 | Barometric altitude above sea level (ISA ref: 1013.25 hPa) |
| `RALT` | m | BME280 | Relative altitude above takeoff point (`BALT − BALT₀`) |
| `VSPD` | m/s | BME280 | Vertical speed (rate of change of `BALT`) |
| `BATT` | V | INA226 | Battery voltage; Drone 1 only |
| `CURR` | A | INA226 | Current draw; Drone 1 only |
| `PWR` | mW | INA226 | Power consumption; Drone 1 only |

The receiver appends link quality on the line immediately after each packet:
```
RSSI: -87.50 dBm, SNR: 6.25 dB
```

> **Note:** `BALT` is independent of GPS and is always present when the BME280 is online. The ground station must not gate its display on `FIX=1`.

---

## Dependencies

### Arduino / PlatformIO

| Library | Used by |
|---|---|
| [RadioLib](https://github.com/jgromes/RadioLib) | All three firmwares |
| [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) | Both transmitters |
| [Adafruit BME280](https://github.com/adafruit/Adafruit_BME280_Library) | Drone 1 |
| [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) | Drone 1 |
| [INA226_WE](https://github.com/wollewald/INA226_WE) | Drone 1 |

Install via Arduino Library Manager or PlatformIO's `lib_deps`.

### Python (ground station)

Python 3.8 or later required.

```
pip install pyserial tkintermapview
```

`tkinter` is included with most Python distributions. On Linux it may need:
```
sudo apt install python3-tk
```

---

## Flashing the Firmware

1. Open the relevant `.ino` file in Arduino IDE (2.x recommended).
2. Install board support:
   - **Heltec ESP32 V3**: add `https://resource.heltec.cn/download/package_heltec_esp32_index.json` to Additional Boards Manager URLs, then install *Heltec ESP32 Series Dev-boards*.
   - **XIAO ESP32-S3**: install *esp32 by Espressif* from Boards Manager.
3. Select the correct board and port, then upload.

> The Heltec ESP32 V3 has the SX1262 connected internally — no external wiring needed for LoRa. The GPS, BME280, and INA226 connect via the exposed GPIO headers; pin assignments are defined at the top of each firmware file.

---

## Running the Ground Station

1. Flash `telemetry_receiver.ino` to the receiver and connect it via USB.
2. Run:
   ```
   python main.py
   ```
3. Select the receiver's COM port from the dropdown and click **Connect**.
4. Drone panels appear automatically as packets are received. The map updates in real time once a GPS fix is acquired.

---

## PCB

The `hardware/` folder contains the Altium Designer project for the Drone 1 custom PCB, which integrates the Heltec ESP32 V3, Uputronics MAX-M10S GPS breakout, BME280, and INA226 onto a single board suitable for mounting on a drone frame. Exported Gerbers are provided in `hardware/gerbers/` for manufacture without Altium.

---

## Known Limitations

- `BALT` uses the ISA standard atmosphere reference (1013.25 hPa). Absolute barometric altitude accuracy degrades with weather; relative altitude (`RALT`) is unaffected as the offset cancels at takeoff.
- LoRa at SF7/BW250 gives an effective range of approximately 500 m in urban/suburban environments. Increase spreading factor (at the cost of airtime and update rate) for longer range.
- The CSMA scheme reduces but does not eliminate packet collisions. At 5 Hz per drone, occasional dropped packets are expected.
- The Python GUI requires a display — it cannot run headless.
