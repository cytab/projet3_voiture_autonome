#!/usr/bin/env python



import rospy

import numpy as np
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt

import sys

cmd_pub = None
enable = True


def PD(old ,  new , num_line , K_two_line=5 , K_one_line=1) :
    if num_line == 2 : 
        kp = K_two_line
    else:
        Kp = K_one_line
    
    deviation = new-old



def callback(msg) : 
    cmd=Twist()
    global enable
    if msg.data == False : 
        enable = True
        cmd.linear.x = 0.2
        cmd_pub(cmd)


def callback_ang(angle) :
     





def main(args):
    rospy.init_node('control')

    #pub = rospy.Publisher("/traitement/error_manager",Bool,queue_size=10)


    cmd_pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)
    
    manage_sub = rospy.Subscriber("/traitement/error_manager", Bool, callback)

    anagle_sub= rospy.Subscriber("/control/angle",Point, callback_ang)

    
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)