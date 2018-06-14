#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Alex Wang'


import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import pyaudio
import wave
import sys


class Communicate(object):
    def __init__(self):
        rospy.init_node("wav",anonymous=True)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.p.get_format_from_width(2),
                channels=1,
                rate=16000,
                input=True,
                output=False,
                frames_per_buffer=640)
        self.output_buffer =""
        self.communicate_sub = rospy.Subscriber('communicate',Bool,self.cb)
        self.wav_pub = rospy.Publisher('wav_data',String, queue_size=1000)
        self.status = -1 


    def cb(self,msg):
        self.status = msg.data
        


if __name__ == '__main__':
    c = Communicate()
    while not rospy.is_shutdown():
        if c.status == False:
            rospy.loginfo('quit...')
            #c.stream.stop_stream()
            #c.stream.close()
            #c.p.terminate()
            wav = String()
            wav.data = c.output_buffer
            c.wav_pub.publish(wav)
            c.status = -1
        elif c.status == True:
            rospy.loginfo('message in...')
            while c.status:
                data = c.stream.read(640)
                c.output_buffer += data
    rospy.spin()
    
 