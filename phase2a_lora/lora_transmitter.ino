/*

Program to establish a publisher/subscriber connection between a Nano 33 BLE and a Bangle.js 2
Date: 21/02/2024
Developed by: ACFR

IMPORTANT: BLE GATT HRM Service app *MUST* be installed and running. This service advertises HRM data continuously.
Link: https://banglejs.com/apps/?id=bootgatthrm

*/

#include <ArduinoBLE.h>
#include <avr/dtostrf.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

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

unsigned long previousMillis = 0; // Variable to store the last time the action was performed
const unsigned int interval = 2000; // Interval in milliseconds

// Define pins used by LoRa module
#define ss 4
#define reset 2
#define dio0 3

void setup() {
  Serial.begin(19200);
  while (!Serial)
    ;
  Serial.println("==================================");

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

  // Initialise LoRa
  LoRa.setPins(ss, reset, dio0);
  while (!LoRa.begin(433E6)) {
    Serial.println("Connecting to LoRa...");
    delay(1000);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa initialised successfully.");

  // Wait for the Bangle.js device to be found
  BLE.scanForAddress("f0:2e:68:49:68:c5");
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
    BLE.scanForAddress("f0:2e:68:49:68:c5");
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


  // As long as the connection is maintained, print every update published by the Bangle
  unsigned long currentMillis;
  char myID[] = "ID01";
  uint8_t bpm = 0;
  float lat_v = 0.0, lon_v = 0.0;
  char lat_array[12], lon_array[12];
  uint8_t time_v[13];
  char time_array[14];
  uint8_t motion_flags[3];
  int8_t temp = 25;
  uint8_t humid = 90;
  int8_t heat_index = 29;
  uint16_t voc = 100, co2 = 90, pm25 = 100;
  uint32_t pressure_v = 0;

  bool first_fix_found = false, new_info = false;

  while (peripheral.connected()) {
    currentMillis = millis(); // Get the current time

    if (hrmCharacteristic.valueUpdated()) {
      new_info = true;
      int32_t value = 0;
      hrmCharacteristic.readValue(value);
      bpm = value >> 8; // bit shift to get only the bpm
    }

    if (lat.valueUpdated() || lon.valueUpdated()) {
      first_fix_found = true;
      new_info = true;
    
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

      time.readValue(time_v, 13);

      // convert time from int array to char array
      for (int i = 0; i < 13; i++) {
          time_array[i] = time_v[i] + ASCII_ZERO;
      }
      // null-terminate the char array
      time_array[13] = 0;
    }

    if (accel.valueUpdated()) {
      new_info = true;
      accel.readValue(motion_flags, 3);
    }

    if (pressure.valueUpdated()) {
      new_info = true;
      pressure.readValue(pressure_v);
    }

    // Check if INTERVAL ms have elapsed since the last action
    if (first_fix_found && new_info && (currentMillis - previousMillis >= interval)) {
      // Save the current time for the next iteration
      previousMillis = currentMillis;

      /*
        LoRa packet format
        01,+123.123456,-123.123456,1234567890123,220,-99,90,-99,101300,1000,2000,1000,0,0,0
      */
      char packet[86];
      heat_index = heatIndex(temp, humid);
      sprintf(packet, "%s,%s,%s,%s,%u,%d,%u,%d,%u,%u,%u,%u,%u,%u,%u", myID, lat_array, lon_array, time_array, bpm, temp, humid, heat_index, pressure_v, voc, co2, pm25, motion_flags[0], motion_flags[1], motion_flags[2]);
      Serial.println("Sending packet");
      sendLoRa(packet);

      new_info = false;
    }
  }
  Serial.println("- Peripheral device disconnected!");
}

int8_t heatIndex(int8_t temperature, uint8_t humidity) {
  float c1 = -8.78469475556;
  float c2 = 1.61139411;
  float c3 = 2.33854883889;
  float c4 = -0.14611605;
  float c5 = -0.012308094;
  float c6 = -0.0164248277778;
  float c7 = 0.002211732;
  float c8 = 0.00072546;
  float c9 = -0.000003582;
  
  float T = temperature;
  float R = humidity;

  float HI = c1 + (c2 * T) + (c3 * R) + (c4 * T * R) + (c5 * T * T) + (c6 * R * R) + (c7 * T * T * R) + (c8 * T * R * R) + (c9 * T * T * R * R);

  return round(HI);
}

void sendLoRa(const char *packet) {
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
}
