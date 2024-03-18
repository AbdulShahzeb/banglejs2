/*

Program to establish a publisher/subscriber connection between a Nano 33 BLE and a Bangle.js 2
Date: 21/02/2024
Developed by: ACFR

*/

#include <ArduinoBLE.h>
#include <avr/dtostrf.h>
#include <SPI.h>
#include <math.h>

// Blue LED Pin
#define BLUE 24

// Define the UUIDs
#define HR_SERVICE_UUID "180D"
#define HR_CHARACTERISTIC_UUID "2A37"

#define GPS_SERVICE_UUID "1819"
#define LAT_UUID "2AAE"
#define LON_UUID "2AAF"
#define TIME_UUID "2A2B"

#define MOTION_SERVICE_UUID "FFA0"
#define ACCEL_UUID "FFA3"

#define BARO_SERVICE_UUID "181A"
#define PRESSURE_UUID "2A6D"

#define GPS_PRECISION 6
#define ASCII_ZERO 48

// default values for loop variables
unsigned long previousMillis = 0; // Variable to store the last time the action was performed
unsigned long currentMillis;
const unsigned int interval = 1000; // Interval in milliseconds
char myID[] = "ID01";
uint8_t bpm = 0;
float lat_v = 0.0, lon_v = 0.0;
char lat_array[12], lon_array[12];
uint8_t time_v[10];
char time_array[11];
uint8_t motion_flags[3];
uint32_t pressure_v = 0;
bool first_fix_found = false, sentPacket = false;


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(BLUE, OUTPUT);
  digitalWrite(BLUE, HIGH);

  // Initialize the BLE
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1)
      ;
  }

  // Scan for nearby BLE devices
  BLE.setLocalName("Nano 33 BLE (Central)");
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE");
  Serial.println(" ");


  // Wait for the Bangle.js device to be found
  BLE.scanForAddress("ea:a2:de:14:81:4c");
  BLEDevice peripheral = BLE.available();
  while (!peripheral) {
    peripheral = BLE.available();
    Serial.println("No BLE device found");
    delay(1000);
  }
}

void loop() {
  connectToPeripheral();
}

void connectToPeripheral() {
  BLEDevice peripheral;

  Serial.println("- Discovering peripheral device...");

  // scan for address of Banglejs2
  do {
    BLE.scanForAddress("ea:a2:de:14:81:4c");
    peripheral = BLE.available();
  } while (!peripheral);

  // Print peripheral info
  if (peripheral) {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

// Function to control the Bangle
void controlPeripheral(BLEDevice peripheral) {
  Serial.println("- Connecting to peripheral device...");

  // Connect to Bangle
  if (peripheral.connect()) {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
    digitalWrite(BLUE, LOW);
  } else {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  // Discover the Bangle's attributes (HRM, GPS, Accel etc. whatever it is advertising)
  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  // Print the advertised service UUIDs, if present
  if (peripheral.hasAdvertisedServiceUuid()) {
    Serial.print("Service UUIDs: ");
    for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
      Serial.print(peripheral.advertisedServiceUuid(i));
      Serial.print(" ");
    }
    Serial.println();
  }


  // Subscribe to the HRM characteristic (topic)
  BLECharacteristic hrmCharacteristic = peripheral.characteristic(HR_CHARACTERISTIC_UUID);
  if (!hrmCharacteristic) {
    Serial.println("* Peripheral device does not have hrm_type characteristic!");
    peripheral.disconnect();
    return;
  } else if (!hrmCharacteristic.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable hrm_type characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!hrmCharacteristic.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to HRM!");


  // Subscribe to the GPS characteristic (topic)
  BLECharacteristic lat = peripheral.characteristic(LAT_UUID);
  if (!lat) {
    Serial.println("* Peripheral device does not have lat characteristic!");
    peripheral.disconnect();
    return;
  } else if (!lat.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable lat characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!lat.subscribe()) {
    Serial.println("Lat subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to Lat!");

  BLECharacteristic lon = peripheral.characteristic(LON_UUID);
  if (!lon) {
    Serial.println("* Peripheral device does not have lon characteristic!");
    peripheral.disconnect();
    return;
  } else if (!lon.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable lon characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!lon.subscribe()) {
    Serial.println("lon subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to lon!");

  BLECharacteristic time = peripheral.characteristic(TIME_UUID);
  if (!time) {
    Serial.println("* Peripheral device does not have time characteristic!");
    peripheral.disconnect();
    return;
  } else if (!time.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable time characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!time.subscribe()) {
    Serial.println("time subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to time!");

  BLECharacteristic accel = peripheral.characteristic(ACCEL_UUID);
  if (!accel) {
    Serial.println("* Peripheral device does not have accel characteristic!");
    peripheral.disconnect();
    return;
  } else if (!accel.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable accel characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!accel.subscribe()) {
    Serial.println("accel subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to accel!");

  BLECharacteristic pressure = peripheral.characteristic(PRESSURE_UUID);
  if (!pressure) {
    Serial.println("* Peripheral device does not have pressure characteristic!");
    peripheral.disconnect();
    return;
  } else if (!pressure.canSubscribe()) {
    Serial.println("* Peripheral does not have a subscribable pressure characteristic!");
    peripheral.disconnect();
    return;
  }
  if (!pressure.subscribe()) {
    Serial.println("pressure subscription failed!");
    peripheral.disconnect();
    return;
  }
  Serial.println("Subscribed to pressure!");


  // As long as the connection is maintained, transmit every update published by the Bangle
  Serial.println("Reading biometric data...");

  while (peripheral.connected()) {
    currentMillis = millis(); // Get the current time

    if (hrmCharacteristic.valueUpdated()) {
      int32_t value = 0;
      hrmCharacteristic.readValue(value);
      bpm = value >> 8; // bit shift to get only the bpm
    }

    if (lat.valueUpdated() || lon.valueUpdated()) {
      first_fix_found = true;
    
      int32_t value = 0;
      lat.readValue(value);
      // convert back from int to float and then to char array
      lat_v = value * pow(10, -GPS_PRECISION);
      dtostrf(lat_v, 11, 6, lat_array);

      value = 0;
      lon.readValue(value);
      // convert back from int to float and then to char array
      lon_v = value * pow(10, -GPS_PRECISION);
      dtostrf(lon_v, 11, 6, lon_array);

      time.readValue(time_v, 10);

      // convert time from int array to char array
      for (int i = 0; i < 10; i++) {
          time_array[i] = time_v[i] + ASCII_ZERO;
      }
      // null-terminate the char array
      time_array[10] = 0;
    }

    if (accel.valueUpdated()) {
      accel.readValue(motion_flags, 3);
    }

    if (pressure.valueUpdated()) {
      pressure.readValue(pressure_v);
    }

    if (first_fix_found && (currentMillis - previousMillis >= interval)) {
      previousMillis = currentMillis;
      /*
        LoRa packet format
        01,+123.123456,-123.123456,1234567890123,220,-99,90,-99,101300,1000,2000,1000,0,0,0
      */
      char packet[52];
      sprintf(packet, "%s,%s,%s,%s,%u,%u,%u,%u,%u", myID, lat_array, lon_array, time_array, bpm, pressure_v, motion_flags[0], motion_flags[1], motion_flags[2]);
      Serial.print("Sending packet: ");
      Serial.println(packet);
      if (!sentPacket) {
        Serial1.print(packet);
        Serial1.flush();
        sentPacket = true;
      }

      if (Serial1.available()) {
        Serial1.readString();
        sentPacket = false;
      }
    }
  }
  Serial.println("- Peripheral device disconnected!");
}


