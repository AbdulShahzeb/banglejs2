#include <SPI.h>
#include <LoRa.h>

// Define pins used by LoRa module
#define ss 4
#define reset 2
#define dio0 3

void setup() {
  Serial.begin(19200);
  while (!Serial)
    ;
  Serial.println("==================================");

  // Initialise LoRa
  LoRa.setPins(ss, reset, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println("Connecting to LoRa...");
    delay(1000);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa initialised successfully.");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
