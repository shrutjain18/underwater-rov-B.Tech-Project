#include <Arduino.h>

#define RADIO_RX_PIN 18  // Connects to Air Radio Pin 3 (Radio TX)
#define RADIO_TX_PIN 17  // Connects to Air Radio Pin 2 (Radio RX)

HardwareSerial RadioSerial(1);

void setup() {
  // USB Serial Monitor (set to 57600 to match the radio speed)
  Serial.begin(57600);
  
  // UART1 connected to the 433MHz Air Radio
  RadioSerial.begin(57600, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN);
}

void loop() {
  // 1. Air -> Ground: Anything typed in ESP32-S3 Terminal goes out over the radio
  while (Serial.available()) {
    RadioSerial.write(Serial.read());
  }

  // 2. Ground -> Air: Anything typed in Ground Radio Terminal prints on ESP32-S3 Terminal
  while (RadioSerial.available()) {
    Serial.write(RadioSerial.read());
  }
}
