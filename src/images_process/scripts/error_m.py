#!/usr/bin/env python



import rospy

import numpy as np
import time
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt

import sys
enable = False
cmd_pub = None

def callback(mng):
    print(mng.data)
    if mng.data == True :
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z =  0.0
        cmd_pub.publish(cmd)
        '''
        if (math.sqrt(cmd.angular.x**2+cmd.angular.y**2+cmd.angular.z**2))!=0 :
            cmd.angular.z = cmd.angular.z - (2/3)*cmd.angular.z
            n = 0
            cmd_pub.publish(cmd)
            while n==20:
                n=n+1

            cmd.angular.z =  0.0
        '''
      #  cmd_pub.publish(cmd)
       



def main(args):
    rospy.init_node('error_manager')
    global cmd_pub

    #pub = rospy.Publisher("/traitement/error_manager",Bool,queue_size=10)

    cmd_pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)
    
    manage_sub = rospy.Subscriber("/traitement/error_manager", Bool, callback)

    
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)