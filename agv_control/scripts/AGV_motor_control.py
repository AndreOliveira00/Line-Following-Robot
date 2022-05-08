#!/usr/bin/env python2

import math
import time
import rospy
from geometry_msgs.msg import Twist 
import wiringpi
import numpy as np 

M1 = 28
M2 = 29
M3 = 22
M4 = 23
PWMA = 26
PWMB = 25
#OUTPUT = 1
BaseSpeed = 100
BASE = math.pi/4


def callback(data):
    rospy.loginfo("linear.x %f",data.linear.x)
    rospy.loginfo("angular.z %f",data.angular.z)

    # velocity gains
    

    if data.angular.z > 0.7:
        data.angular.z=0.7
    elif data.angular.z < - 0.7:
        data.angular.z = -0.7


    if(data.linear.x > 0 ): #frente
  	
        if(data.angular.z == 0):  #frente i
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.HIGH)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.HIGH)
            RA=int(BaseSpeed*data.linear.x)
            RB= int(BaseSpeed*data.linear.x)
            wiringpi.softPwmWrite(PWMA , RA)
            wiringpi.softPwmWrite(PWMB , RB)
            rospy.loginfo("Frente: PWMA -> %d   PWMB -> %d"%(RA,RB))
		
        elif(data.angular.z > 0): #frente esquerda u
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.HIGH)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.HIGH)
            RA=int(BaseSpeed*data.linear.x*np.cos(BASE+data.angular.z))
            RB=int(BaseSpeed*data.linear.x*np.sin(BASE+data.angular.z))
            wiringpi.softPwmWrite(PWMA , RA )
            wiringpi.softPwmWrite(PWMB , RB ) 	
            rospy.loginfo("Frente Esquerda: PWMA -> %d   PWMB -> %d"%(RA,RB))

            print(RA,RB)
		
        elif(data.angular.z < 0):#frente direita o
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.HIGH)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.HIGH)
            RA=int(BaseSpeed*data.linear.x*np.cos(BASE+data.angular.z))
            RB=int(BaseSpeed*data.linear.x*np.sin(BASE+data.angular.z))
            wiringpi.softPwmWrite(PWMA , RA)
            wiringpi.softPwmWrite(PWMB , RB)	
            rospy.loginfo("Frente Direita: PWMA -> %d   PWMB -> %d"%(RA,RB))			
            
    
    elif(data.linear.x < 0 ): #tras
  	
        if(data.angular.z==0):  #tras i
            wiringpi.digitalWrite(M1, wiringpi.HIGH)
            wiringpi.digitalWrite(M2, wiringpi.LOW)
            wiringpi.digitalWrite(M3, wiringpi.HIGH)
            wiringpi.digitalWrite(M4, wiringpi.LOW)
            wiringpi.softPwmWrite(PWMA , -int(BaseSpeed*data.linear.x) )
            wiringpi.softPwmWrite(PWMB , -int(BaseSpeed*data.linear.x) )
            rospy.loginfo("Tras")
		
        elif(data.angular.z>0): #tras Direita u
            wiringpi.digitalWrite(M1, wiringpi.HIGH)
            wiringpi.digitalWrite(M2, wiringpi.LOW)
            wiringpi.digitalWrite(M3, wiringpi.HIGH)
            wiringpi.digitalWrite(M4, wiringpi.LOW)
            wiringpi.softPwmWrite(PWMA , abs(int(BaseSpeed*data.linear.x*np.cos(BASE-data.angular.z))))
            wiringpi.softPwmWrite(PWMB , int(BaseSpeed*data.linear.x*np.sin(BASE-data.angular.z)) )	
            rospy.loginfo("Tras Direita")
            rospy.loginfo(int(BaseSpeed*data.linear.x*np.cos(-data.angular.z)))
		
        elif(data.angular.z<0):#tras esquerda o
            wiringpi.digitalWrite(M1, wiringpi.HIGH)
            wiringpi.digitalWrite(M2, wiringpi.LOW)
            wiringpi.digitalWrite(M3, wiringpi.HIGH)
            wiringpi.digitalWrite(M4, wiringpi.LOW)
            wiringpi.softPwmWrite(PWMA , int(BaseSpeed*data.linear.x*np.cos(BASE-data.angular.z)))
            wiringpi.softPwmWrite(PWMB , abs(int(BaseSpeed*data.linear.x*np.sin(BASE-data.angular.z))))				
            rospy.loginfo("Tras Esquerda")
            rospy.loginfo(int(BaseSpeed*data.linear.x*np.cos(-data.angular.z)))

    elif (data.linear.x == 0 ): # parar k
        if(data.angular.z==0):  # direita j
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.LOW)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.LOW)
            wiringpi.softPwmWrite(PWMA , 0 )
            wiringpi.softPwmWrite(PWMB , 0 )
            rospy.loginfo("Stop")
		
        elif(data.angular.z>0): #esquerda l
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.HIGH)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.LOW)
            wiringpi.softPwmWrite(PWMA , 0)
            wiringpi.softPwmWrite(PWMB , abs(int(100*np.sin(data.angular.z)*0.5)) )	
            rospy.loginfo("Esquerda")
		
        elif(data.angular.z<0):#tras direita o
            wiringpi.digitalWrite(M1, wiringpi.LOW)
            wiringpi.digitalWrite(M2, wiringpi.LOW)
            wiringpi.digitalWrite(M3, wiringpi.LOW)
            wiringpi.digitalWrite(M4, wiringpi.HIGH)
            wiringpi.softPwmWrite(PWMA , abs(int(100*np.sin(data.angular.z)*0.5)))
            wiringpi.softPwmWrite(PWMB , 0)				
            rospy.loginfo("Direita")


def start():
    rospy.init_node('motor_control', anonymous=False)
    pin_setup()
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

def pin_setup():
    wiringpi.wiringPiSetup()
    wiringpi.pinMode(M1 , wiringpi.OUTPUT)
    wiringpi.pinMode(M2 , wiringpi.OUTPUT )
    wiringpi.pinMode(M3 , wiringpi.OUTPUT )
    wiringpi.pinMode(M4 , wiringpi.OUTPUT )
    wiringpi.softPwmCreate(PWMA , 0, 100)
    wiringpi.softPwmCreate(PWMB , 0, 100)
 

if __name__ == '__main__':
    try:
        rospy.loginfo('START')
        start()
        rospy.spin()
    except rospy.ROSInterruptException:


        pass

