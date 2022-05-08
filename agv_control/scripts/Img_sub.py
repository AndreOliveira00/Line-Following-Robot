#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage
import pickle
import os

log_n=0

def callback(data):
    global log_n
    rospy.loginfo("\nI heard :\n")
    with open(os.path.expanduser('~')+"/logs/IMG/img_%d"%log_n, "wb") as fp:
        pickle.dump(data.data, fp)

    log_n=log_n+1
    print("Log number",log_n)

def start():

    rospy.init_node('Log_maker', anonymous=False)
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:


        pass
