/*

Program to transmit biom and env data via LoRa
Date: 18/03/2024
Developed by: ACFR

*/

#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <math.h>
SoftwareSerial link(5,6); // Rx, Tx


unsigned long previousMillis = 0; // Variable to store the last time the action was performed
const unsigned int interval = 1500; // Interval in milliseconds

// Define pins used by LoRa module
#define CS 3
#define RST 4
#define G0 2

//configure dht11
#include "DHT.h"
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//configure CCS811 air sensor
#include "CCS811.h" //CCS811 sensor(&Wire, /*IIC_ADDRESS=*/0x5A);
CCS811 sensor;

//configure dust sensor
int measurePin = A0;
int ledPower = 7;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;


// default values for loop variables
unsigned long currentMillis;
int temp = 0, humid = 0;
int heat_index = 0;
uint16_t voc = 0, co2 = 0;
int pm25 = 0;


void setup() {
  Serial.begin(115200);
  link.begin(9600);


  // Initialise LoRa
  LoRa.setPins(CS, RST, G0);
  while (!LoRa.begin(433E6)) {
    Serial.println("Connecting to LoRa...");
    delay(1000);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa initialised successfully.");

  // setup dust sensor
  pinMode(ledPower,OUTPUT);

  // setup air quality sensor ----------
  while(sensor.begin() != 0){
    Serial.println("Failed to init air quality sensor.");
    delay(1000);
  }
  sensor.setMeasCycle(sensor.eCycle_250ms);


  //setup dht11
  dht.begin(); // initialize the sensor

  delay(1000);// delay to allow stability.
}


void loop() {
  // receive Bangle updates, then append Weather updates and send LoRa packet
    currentMillis = millis(); // Get the current time

    // Check if INTERVAL ms have elapsed since the last action
    if (currentMillis - previousMillis >= interval) {
    // Save the current time for the next iteration
    previousMillis = currentMillis;
    link.print("Ready");
    do {
      delay(100);
    } while (!link.available());

    char biometrics[52];
    strcpy(biometrics, link.readString().c_str());

    if (biometrics[0] != 'I') {
      return;
    }

    /*
      LoRa packet format
      01,+123.123456,-123.123456,1234567890123,220,-99,90,-99,101300,1000,2000,1000,0,0,0
    */
    char packet[86];
    temp = int(dht.readTemperature());
    humid = int(dht.readHumidity());
    heat_index = heatIndex(temp, humid);
    pm25 = getDust();
    voc = sensor.getTVOCPPB();
    co2 = sensor.getCO2PPM();
    sensor.writeBaseLine(0x847B);
    sprintf(packet, "%s,%d,%d,%d,%u,%u,%d", biometrics, temp, humid, heat_index, voc, co2, pm25);
    Serial.print("Sending packet: ");
    Serial.println(packet); // REMOVE THIS LINE LATER
    sendLoRa(packet);

  }
}

int heatIndex(int temperature, int humidity) {
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

int getDust(){
  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024.0);
  dustDensity = 170*calcVoltage - 0.1;

  if ( dustDensity < 0.0)
  {
    dustDensity = 0.00;
  }
  return int(dustDensity);  
}
