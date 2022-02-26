#!/usr/bin/env python

import rospy

from steve_serial.msg import testmsg 


if __name__ == '__main__':

    rospy.init_node('arduino_serial', anonymous=True)

    rospy.Subscriber('test_msg', testmsg, self.sendMultipleGPSGoals)

    MultipleGpsGoals()
    rospy.spin()


