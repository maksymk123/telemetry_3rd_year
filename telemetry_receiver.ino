#include <RadioLib.h>

// ================== LoRa pins (SX1262) ==================
#define LORA_NSS_PIN   8
#define LORA_DIO1_PIN  14
#define LORA_RST_PIN   12
#define LORA_BUSY_PIN  13

SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);

// ================== Interrupt-driven RX flag ==================
// RadioLib calls this ISR when a packet arrives.
// We just set a flag — never do Serial.print inside an ISR.
volatile bool packetReceived = false;

void IRAM_ATTR onPacketReceived() {
  packetReceived = true;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("LoRa RX starting..."));

  // LoRa settings MUST match both transmitters exactly.
  int16_t state = radio.begin(868.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("radio.begin() failed, code ")); Serial.println(state);
    while (true) { delay(1000); }
  }
  radio.setSpreadingFactor(7);
  radio.setBandwidth(250.0);
  radio.setCodingRate(5);
  radio.setOutputPower(14);   // not used in RX, but keeps config explicit

  // Attach interrupt to DIO1 and start listening.
  // startReceive() puts the radio into continuous RX mode — it never blocks.
  radio.setPacketReceivedAction(onPacketReceived);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("startReceive() failed, code ")); Serial.println(state);
    while (true) { delay(1000); }
  }

  Serial.println(F("LoRa RX ready (non-blocking, interrupt-driven)."));
}

// ================== LOOP ==================
void loop() {
  // Check the interrupt flag set by onPacketReceived().
  // This pattern never blocks — the loop runs freely at full speed.
  if (packetReceived) {
    packetReceived = false;   // clear flag immediately

    String str;
    int16_t state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      // Print payload — GUI reads this line
      Serial.println(str);

      // Print link quality on the next line — GUI attaches it to the last drone
      Serial.print(F("RSSI: "));
      Serial.print(radio.getRSSI(), 2);
      Serial.print(F(" dBm, SNR: "));
      Serial.print(radio.getSNR(), 2);
      Serial.println(F(" dB"));

    } else {
      Serial.print(F("readData() error, code ")); Serial.println(state);
    }

    // Re-arm the receiver for the next packet.
    // Must be called after every readData() when using interrupt mode.
    radio.startReceive();
  }

  // The loop is intentionally empty otherwise.
  // No delay() — we want to catch the interrupt flag as fast as possible.
}
