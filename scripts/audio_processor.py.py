#!/usr/bin/env python 

#THIS NODE'LL READ RESPEAKER INPUTS OF VAD (VOICE ACTIVITY DETECTION)

import sys
import signal
import webrtcvad
import numpy as np
from mic_array import MicArray
import rospy 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import math


#initialize variables
RATE = 16000
CHANNELS = 4
VAD_FRAMES = 10     # ms
DOA_FRAMES = 200    # ms

pub = rospy.Publisher('angle', Float64, queue_size=10)

def sigint_handler(signal, frame):
    print("Stopping...")
    sys.exit(0)

def main():
    #initialize node
    rospy.init_node('vaddoa_node')  
    vad = webrtcvad.Vad(3)
    speech_count = 0
    chunks = []
    doa_chunks = int(DOA_FRAMES / VAD_FRAMES)

    signal.signal(signal.SIGINT, sigint_handler)

    with MicArray(RATE, CHANNELS, RATE * VAD_FRAMES / 1000)  as mic:
        for chunk in mic.read_chunks():
            # Use single channel audio to detect voice activity
            if vad.is_speech(chunk[0::CHANNELS].tobytes(), RATE):
                speech_count += 1

            #sys.stdout.flush()

            chunks.append(chunk)
            if len(chunks) == doa_chunks:
                if speech_count > (doa_chunks / 2):
                    frames = np.concatenate(chunks)
                    direction = mic.get_direction(frames)
                    print('\n{}'.format(int(direction)))
                    pub.publish(int(direction))

                speech_count = 0
                chunks = []

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
