#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Alex Wang'


import rospy
from std_msgs.msg import Int8

if __name__ == '__main__':
    rospy.init_node("cmd",anonymous=True)
    pub = rospy.Publisher('microphone_mode_cmd',Int8,queue_size=10)
    rospy.sleep(5)
    cmd = Int8(2)
    pub.publish(cmd)
    rospy.sleep(10)
    cmd = Int8(-2)
    pub.publish(cmd)
    rospy.sleep(10)


        
