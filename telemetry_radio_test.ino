#include <Arduino.h>

#define RADIO_RX_PIN 18
#define RADIO_TX_PIN 17
#define RADIO_BAUD   57600

HardwareSerial RadioSerial(1);
unsigned long lastTelemTime = 0;
int packetCount = 0;

void setup() {
  Serial.begin(115200); // For your laptop to read via USB
  RadioSerial.begin(RADIO_BAUD, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN);
  
  Serial.println("\n=== ESP32-S3 Submarine Link Tester ===");
  Serial.println("Listening for Dashboard Commands...");
}

void loop() {
  // 1. Listen for commands from the HTML Dashboard over 433MHz
  while (RadioSerial.available()) {
    String incoming = RadioSerial.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      // Print exactly what arrived from the air to your Arduino Serial Monitor
      Serial.println("[RECEIVED FROM AIR] -> " + incoming);
    }
  }

  // 2. Transmit dummy telemetry (zeros) every 500ms
  if (millis() - lastTelemTime >= 500) {
    lastTelemTime = millis();
    packetCount++;

    // Dummy data packet: Zeros for sensors, but packet count goes up
    String telem = "Depth:0.00,Pitch:0.0,BattV:12.0,FS:0,Pkt:" + String(packetCount) + "\n";
                   
    RadioSerial.print(telem);
    // Also print it locally so you know the ESP32 is transmitting
    Serial.print("[TRANSMITTING] -> " + telem); 
  }
}
