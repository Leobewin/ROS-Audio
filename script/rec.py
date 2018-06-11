#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Alex Wang'


import rospy
from std_msgs.msg import String
import pyaudio
import wave
import sys


def cb(msg):
    rospy.loginfo("ready to write")
    output_wav = wave.open("output.wav", 'w')
    output_wav.setparams((1, 2, 16000, len(msg.data),"NONE","not compressed"))
    output_wav.writeframesraw(msg.data)


if __name__ == '__main__':
    rospy.init_node('rec',anonymous=True)
    rospy.Subscriber('wav_data', String, cb)
    rospy.spin()

