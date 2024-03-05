#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <LoRa.h>

// Define pins used by LoRa module
#define ss 4
#define reset 2
#define dio0 3
#define PACKET_SIZE 88

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher bangle("bangle", &str_msg);

void setup()
{
  // Init ROS node handle
  nh.initNode();
  nh.advertise(bangle);

  // Initialise LoRa
  LoRa.setPins(ss, reset, dio0);
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
      bangle.publish( &str_msg ); 
      nh.spinOnce();
    }
  }
}
