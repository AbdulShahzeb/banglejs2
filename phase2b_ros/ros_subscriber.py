#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from datetime import datetime

nodeName='bangle_processed'
topicName='bangle_raw'

# ID01,+123.123456,-123.123456,1234567890123,220,101300,0,0,0,-99,90,-99,1000,2000,1000
def callbackFunction(msg):
    data = [x.strip() for x in msg.data.split(',')]
    print("Message received from Bangle", data[0])
    print("Latitude:", data[1])
    print("Longitude:", data[2])
    time_str = datetime.fromtimestamp(int(data[3])).strftime("%H:%M:%S")
    print("Time:", time_str)
    print("Heartrate:", data[4], "bpm")
    pressure = int(data[5]) * 0.01
    print(f"Pressure: {pressure:.2f} hPa")
    print("Impact:", bool(int(data[6])), ", NoMotion:", bool(int(data[7])), ", Alert:", bool(int(data[8])))
    print("Temperature:", data[9] + "°C")
    print("Humidity:", data[10] + "%")
    print("Heat Index:", data[11] + "°C")
    print("VOC:", data[12], "ppb")
    print("eCO2:", data[13], "ppm")
    print("Dust:", data[14], "mcg/m3")
    

rospy.init_node(nodeName,anonymous=True)
rospy.Subscriber(topicName, String, callbackFunction)

rospy.spin()