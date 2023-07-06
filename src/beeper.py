#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import wiringpi
from wiringpi import GPIO

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    if data.data is True:
        wiringpi.digitalWrite(16, GPIO.HIGH) # beep
    else:
        wiringpi.digitalWrite(16, GPIO.LOW)  # not beep
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Beeper', anonymous=False)

    wiringpi.wiringPiSetup()
    wiringpi.pinMode(16, GPIO.OUTPUT)

    rospy.Subscriber("beeper", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
