#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <LoRa.h>

// Define pins used by LoRa module
#define CS 3
#define RST 4
#define G0 2
#define PACKET_SIZE 86

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher bangle_raw("bangle_raw", &str_msg);

void setup()
{
  // Init ROS node handle
  nh.initNode();
  nh.advertise(bangle_raw);

  // Initialise LoRa
  LoRa.setPins(CS, RST, G0);
  while (!LoRa.begin(433E6)) {
    delay(1000);
  }

  LoRa.setSyncWord(0xF3);

}

void loop()
{
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet -> read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      char LoRaArr[PACKET_SIZE + 1];
      LoRaData.toCharArray(LoRaArr, sizeof(LoRaArr));
      str_msg.data = LoRaArr;
      bangle_raw.publish( &str_msg ); 
      nh.spinOnce();
    }
  }
}
