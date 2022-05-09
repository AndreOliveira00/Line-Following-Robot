#!/usr/bin/env python


import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import time
import math
import pickle
import os
import cv2
import numpy as np

VERTICAL_ROI = 80
ANGLE_OFFSET = 3

# VEL = 0.17
# P = 0.30
# I = 0.007
# D = 0.70

# Linear = 1.2
# Angular = 0.6
# VEL = 0.25
# P = 0.35
# I = 0.002
# D = 1.5

# Linear = 1.2
# Angular = 0.8
# VEL = 0.25
# P = 0.31
# I = 0.002
# D = 1.7

# Linear = 1.2
# Angular = 0.8
# VEL = 0.3
# P = 0.32
# I = 0.004
# D = 2.22

# JARDA_NA_RETA = 1
# JARDA_NA_CURVA = 0.6
# VEL = 0.25
# P = 0.23
# I = 0.004
# D = 2

JARDA_NA_RETA = 1.1
JARDA_NA_CURVA = 0.8
VEL = 0.2
P = 0.27
I = 0.0030
D = 3.05
log_n=0
CI_VALUE = 3000

class PID:
    """PID controller."""

    def __init__(self, Kp, Ki, Kd):
  
        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_error = 0.0

    def Update(self, error):


        self.Cp = error
        self.Ci += error
        self.Cd = error - self.previous_error
        # print("SOMA CI ->>",self.Ci)
        if self.Ci>CI_VALUE: 
            self.Ci =CI_VALUE
        elif self.Ci<-CI_VALUE:
            self.Ci =-CI_VALUE
        # print("P ->",self.Kp * self.Cp)
        # print("I ->",self.Ki * self.Ci)
        # print("D ->",self.Kd * self.Cd)

        self.previous_error = error

        return (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )

class Follow_Line:

    def __init__(self) :

        rospy.init_node('image_processing', anonymous=False)
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        self.pub2 = rospy.Publisher('processed_image/image/compressed', CompressedImage)
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.pid = PID(P, I, D)
        self.range_view=0

    def main(self):
    
        rospy.spin()
 
    def callback(self,compressed_img):
        #global log_n

        rospy.loginfo("")
        
        #global log_n
        #with open(os.path.expanduser('~')+"/logs/IMG/img_%d"%log_n, "wb") as fp:
        #    pickle.dump(compressed_img.data, fp)

        #log_n=log_n+1
        #print("Log number",log_n)

        np_arr=np.frombuffer(compressed_img.data,np.uint8)
        img=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        
        height, width = img.shape[:2]
        roi = img[(int(height-(VERTICAL_ROI+self.range_view))):int((height-self.range_view)), 0:width]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 0])
        upper_white = np.array([255,255,57])

        median = cv2.medianBlur(hsv,21)
        mask = cv2.inRange(median, lower_white, upper_white)
        
        th2 = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV, 9,15)
        
        mean_point=[]
        for y in range(roi.shape[0]-35,0,-3):
            x=-1
            last_color=0 
            points=[]
            for c in range(0,roi.shape[1],4):
                color=th2[y][c]
                if color== 255 and last_color==0:
                    points.append([c,y])
                last_color = color
            
            if len(points) == 2 and (points[1][0] - points[0][0]) in range(40,200):
         
                cv2.circle(th2, (mean(points[0][0],points[1][0]),y),1, (255, 255, 255), 2)   
                mean_point.append([mean(points[0][0],points[1][0]),y])
            if len(mean_point) > 4:
                break
        if len(mean_point) > 0:
            # if math.dist(mean_point[0],mean_point[-1]) > 20:
            height, width = th2.shape[:2]  
            zero=(int(width/2),height)
            angle=get_angle(zero,mean_point[-1])+90
            cv2.line(th2,tuple(zero),tuple(mean_point[-1]),(255,255,255),2)
            # correction=angle - last_angle
            # last_angle=angle

            correction = self.pid.Update(angle)
            #print("Entrada -->",angle,"|| Saida --> ",correction)
            #"%d deg"%correction
            cv2.putText(img=th2, text="%d deg"%correction, org=(10,20), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 255, 255),thickness=1)
            self.CommandUGV(correction)
    
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', th2)[1]).tostring()
        
    
        
        self.pub2.publish(msg)

    def CommandUGV(self,angle):
     
        twist = Twist()
        # (self.pid.previous_error<15 and self.pid.previous_error>-15):
        if (angle and self.pid.previous_error) < 17 and (angle and self.pid.previous_error) > -17 :
            vel_ratio=JARDA_NA_RETA
            self.range_view=45
        else:
            vel_ratio=JARDA_NA_CURVA
            self.range_view=0

        angular_vel =  0 if(angle<ANGLE_OFFSET and angle>-ANGLE_OFFSET) else math.radians(angle)
        twist.angular.z = angular_vel
        twist.linear.x = VEL*vel_ratio
        self.vel_publisher.publish(twist)

def mean(x1,x2):
    return int((x1+x2)/2)


def get_angle(p1, p2):
    myradians = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
    mydegrees = math.degrees(myradians)
    return mydegrees


def start():
    follow_Line = Follow_Line()
    follow_Line.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:

        pass
         
    