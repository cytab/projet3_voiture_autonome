#!/usr/bin/env python



import rospy
import numpy as np
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from matplotlib import pyplot as plt
import sys

x =1
enable = None
cmd = Twist()
cmd_pub = None
K2l = 0.4
K1l = 0.17

K3l=0.01
K4l = 0.004

'''.
k5l=0.0005
k6l=0.0002
'''
angl_deviation = 0.0
previous_error = 0.0
erreur = 0.0







def callback(data) : 
    global cmd
    global enable
    global x
    enable = data.data
    if enable == False :
        if x ==2 :
            cmd.linear.x = 0.2
            cmd_pub.publish(cmd)
        elif x==1 :
            cmd.linear.x = 0.2
            cmd_pub.publish(cmd)
    elif enable == True :
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        cmd_pub.publish(cmd)
    
        

def a_callback(data):
    global K2l
    global K1l
    global K3l
    global K4l
    global previous_error
    global angl_deviation
    global erreur

    x = data.z
    if x == 2 :
        Kp  = K2l
        Kd = K3l
        print("2l")
    elif x==1 :
        print("1l")
        Kp = K1l
        Kd= K4l
    angl_deviation =  data.x -data.y
    if abs(angl_deviation) > 0.07  : 
        angle_finale = -(Kp*data.x+Kd*(angl_deviation))
        #-previous_error))
       # angl_deviation/abs(angl_deviation))
        #+Ki*erreur)
        '''
        (angl_deviation/abs(angl_deviation)
        '''
        print("grosse variation", angle_finale)
        cmd.angular.z = angle_finale
        cmd_pub.publish(cmd)
    '''
    elif abs(angl_deviation) > 0.8 : 
        angle_finale = -(0.09*data.x+0.08*(angl_deviation))
        #-previous_error))
       # angl_deviation/abs(angl_deviation))
        #+Ki*erreur)
       
        
        (angl_deviation/abs(angl_deviation)
    
        print("dichotomie", angle_finale)
        cmd.angular.z = angle_finale
        cmd_pub.publish(cmd)
    '''
    if abs(angl_deviation) < 0.07 : 
        angle_finale = -(Kp*data.x)
        print(angle_finale)
        cmd.angular.z = angle_finale
        cmd_pub.publish(cmd)


    previous_error = angl_deviation
    erreur = erreur+angl_deviation
        




def main(args):
    global cmd_pub
    cmd = Twist()

    rospy.init_node('controlleur')

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    manage_sub = rospy.Subscriber("/traitement/error_manager", Bool, callback)

    angle_sub = rospy.Subscriber("/control/angle",Point, a_callback)

    rospy.spin()


'''
    rate = rospy.Rate(100000000000)

    while not rospy.is_shutdown() :

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = -0.0
        cmd_pub.publish(cmd)




        

        rate.sleep()
'''
    
    


if __name__ == '__main__':
    main(sys.argv)