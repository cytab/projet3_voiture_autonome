#!/usr/bin/env python



import rospy
import cv2
import numpy as np
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import sys


#--------tunning canny ---------------------------



rho = 2
theta=1*np.pi/180
threshold = 55
min_line_length = 30
max_line_gap = 20







#----------------------------------------------------

#----------region of interest ----------------------
trap_bottom_width = 1
trap_top_width = 0.9
trap_height = 0.4
#-------------------------------------------------
bridge=CvBridge() ; 

#-----------------filtre alpha beta--------------------
alpha = 0.75
beta = 0.001
prediction = 0.0
estimate = 0.0
sample_time = 0.033
rate = 1

x_offset_old = 1.0
Y_offset_old = 1.0
s1_time = 0.0
x = False ; 
t1 = 0.0
#------------------------------------------------------






def give_angle(image, average_line):
    global x
    global s1_time 
    global x_offset_old
    global Y_offset_old
    global prediction
    global rate
    global estimate 
    global sample_time
    global alpha
    global beta

    error_manager = Bool()
    height = image.shape[0]
    width = image.shape[1]

    if len(average_line) ==1 :
        x1, _, x2, _ = average_line[0]
        x_offset = x2-x1
        y_offset = int(height/2)
        x_offset_old =x_offset
        Y_offset_old = y_offset
    elif len(average_line) ==2 : 
        _, _, left_x2, _= average_line[0]
        _, _, right_x2, _= average_line[1]
        mid = int(width/2)
        x_offset = (left_x2+right_x2)/2 - mid
        y_offset = int(height/2)
        x_offset_old =x_offset
        Y_offset_old = y_offset
    '''
    global s1_time 
    s1_time= time.time()
    global t1
    t1 =time.time()-s1_time
    '''
    #t1 >1 and 
    '''
    #---- alpha beta filter appplique sur le x_offset de la ligne du milieu  --------------------------------------------------
    #prediciton part

    rate = rate
    prediction =estimate + rate*sample_time

    #estimate
    
    rate = rate + beta*(x_offset_old - (prediction/sample_time))
    estimate = prediction + alpha*(x_offset_old-prediction)


    valeur_predite = estimate
    print("valeuuuuuuuurrrrr estimate : " , estimate) 
    vitesse_predite = rate 
'''
    #----------------------------------------------------------------------------------------------------------------------------
    if len(average_line) == 0 and x == False:
       
       s1_time= time.time()
       x= True 
    elif  len(average_line) != 0 :
        s1_time = 0.0
        x = False ; 
        

    global t1
    t1 =time.time()-s1_time

    
    '''
    if t1 < 3 and len(average_line) == 0  :
        x_offset = Y_offset_old
        y_offset = Y_offset_old
        error_manager.data = False
        print("temps correct")
        pub.publish(error_manager)
        t1 =time.time()-s1_time



    if len(average_line) == 0  and t1 > 3 and t1 < 8 :
        print("temps estimation")
        x_offset = valeur_predite
        y_offset = Y_offset_old
        error_manager.data = False
        pub.publish(error_manager)
        t1 =time.time()-s1_time
'''
    if   t1 > 7 and len(average_line) == 0 :
       error_manager.data = True
       print("arret auto")

       t1 =time.time()-s1_time

   #if error_manager.data == False :
    angle_rad = math.atan(x_offset/y_offset)
    angle_deg = int (angle_rad* 180.0 / math.pi)
    angle_f = angle_deg + 90
    return angle_f






def display_heading_line(image, angle , color = (0,0,255),line_w = 2) :
    heading_image = np.zeros_like(image)
    height = image.shape[0]
    width = image.shape[1]
    
    angle_F = angle / 180.0 * math.pi

    x1 = int(width/2)
    y1=height
    x2 = int(x1 - height/ 2 / math.tan(angle_F))
    y2 = int(height/2)

    cv2.line(heading_image,(x1,y1),(x2,y2),color,line_w)
    heading_image = cv2.addWeighted(image,0.8,heading_image,1,1)
    return heading_image


def make_coordinates(image, line_parameters):
    ''' global previous_line_parameters
    if line_parameters == None : 
        line_parameters = previous_line_parameters
    ''' 
    height = image.shape[0]
    width  = image.shape[1]
    slope, intercept = line_parameters
    y1 = height
    y2=int(y1*(1-trap_height))
    x1 = max((-width,min(2*width,int(y1-intercept)/slope)))
    x2 = max((-width,min(2*width,int(y2-intercept)/slope)))
    return np.array([x1,y1,x2,y2])




def average_slope_intercept(image,lines):
    lane_lines =[]
    error_manager = Bool()
    if lines is None:
        return lane_lines

    #global s1_old 
    #s1_old= lane_lines
    left_fit= []
    right_fit = []

    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
      #   if x1 ==x2:
      #      slopet = 999.
        slopet = (y2-y1)/(x2-x1)
        if  abs(slopet) < 800 :                       
            parameters = np.polyfit((x1,x2),(y1,y2),1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0 :
                left_fit.append((slope,intercept))
            else:
                right_fit.append((slope,intercept))

    left_fit_average = np.average(left_fit,axis = 0)
    if len(left_fit) >0:
        lane_lines.append(make_coordinates(image,left_fit_average))

    right_fit_average = np.average(right_fit,axis = 0)
    if len(right_fit) >0:
        lane_lines.append( make_coordinates(image,right_fit_average))

    '''
    if len(lane_lines) !=0 :
        error_manager.data = False
        pub.publish(error_manager)
    '''
    return lane_lines



def canny(image):
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny_img=cv2.Canny(blur,200,150)
    return canny_img

def display_lines(image,lines) : 
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line.reshape(4)
            pts = np.array([[x1,y1],[x2,y2]],np.int32)
            cv2.polylines(line_image,[pts],True, (255,255,0),3)

    return line_image



def region_of_interest(image):
    h = image.shape[0]
    w = image.shape[1]
    polygons = np.array([[\
       ((w*(1-trap_bottom_width))//2,h),\
        ((w*(1-trap_top_width))//2,h-h*trap_height),\
        (w-(w*(1-trap_top_width))//2,h-h*trap_height),\
        (w-(w*(1-trap_bottom_width))//2,h) ]]\
        , dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillConvexPoly(mask,polygons,255)
    masked_image =cv2.bitwise_and(image,mask)

    return masked_image







def main(args):
 #   if __name__ == '__main__':

    rospy.init_node('essaie_traitement_filtre')

    cap = cv2.VideoCapture("ROUTE.mp4")
    while(cap.isOpened()) :
        _,image = cap.read()
        try :
            img = bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        (ligne,colonne,channels) = img.shape
        key = cv2.waitKey(10) & 0xFF
        if key == ord("q"):
            raise Exception("on sort")
        img_clone = img.copy()
        canny_image = canny(img_clone)
        cropped_image = region_of_interest(canny_image)
        lines = cv2.HoughLinesP(cropped_image,rho,theta,threshold,np.array([]),minLineLength=min_line_length
         ,maxLineGap=max_line_gap)
        averaged_lines = average_slope_intercept(img_clone,lines)
        line_image = display_lines(img_clone,averaged_lines)
    
        combo_image = cv2.addWeighted(img_clone,0.8,line_image,1,1)
        angle = give_angle(combo_image, averaged_lines)
        final_image= display_heading_line(combo_image , angle)
        cv2.imshow("Afficher camera",final_image)






    

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



if __name__ == '__main__':
    main(sys.argv)
    
