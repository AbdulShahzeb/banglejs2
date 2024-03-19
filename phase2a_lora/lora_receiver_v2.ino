/*

Program to receive LoRa packets from the transmitter
Date: 18/03/2024
Developed by: ACFR

*/

#include <SPI.h>
#include <LoRa.h>

// Define pins used by LoRa module
#define CS 3
#define RST 4
#define G0 2

void setup() {
  Serial.begin(57600);

  // Initialise LoRa
  LoRa.setPins(CS, RST, G0);
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
