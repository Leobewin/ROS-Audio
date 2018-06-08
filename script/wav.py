#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import pyaudio
import wave
import sys


class Communicate(object):
    def __init__(self):
        rospy.init_node("wav",anonymous=True)
        #rospy.Subscriber("communicate",Bool,self.cb)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.p.get_format_from_width(2),
                channels=1,
                rate=16000,
                input=True,
                output=False,
                frames_per_buffer=640)
        self.output_buffer =""
    '''
    def cb(self,msg):
        if(msg.data == False):
            rospy.loginfo("stopping......")
            self.stream.stop_stream()
            self.stream.close()
            output_wav = wave.open("output.wav", 'w')
            output_wav.setparams((1, 2, 16000, len(self.output_buffer),"NONE","not compressed"))
            output_wav.writeframesraw(self.output_buffer)
            self.p.terminate()

        while(msg.data == True):
            data = self.stream.read(640)
            self.output_buffer += data
    '''


            
    
if __name__ == '__main__':
    c = Communicate()
    pub = rospy.Publisher('wav_data',String, queue_size=1000)
    i = 0
    while(i < 100):
        data = c.stream.read(640)
        c.output_buffer += data
        i += 1
    c.stream.stop_stream()
    c.stream.close()
    wav = String()
    wav.data = c.output_buffer
    pub.publish(wav)
    print(type(c.output_buffer))
    rospy.spin()
    

 