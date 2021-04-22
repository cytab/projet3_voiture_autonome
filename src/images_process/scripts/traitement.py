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
from matplotlib.animation import FuncAnimation
from cv_bridge import CvBridge, CvBridgeError
import sys

'''
#--------------------------------------------------------------------------------------
# definition des variable
#-------------------------------------------------------------------------------------

ymin = 80
ymax =81
xmin1=0
xmax1=90
xmin2=230
xmax2=320
#previous_line_parameters = 0,0
'''
#-------------------------------------------------houghlines---------------------------------------


thetaf=[]
Time= time.time()





rho = 2
theta=1*np.pi/180
threshold = 55
min_line_length = 30
max_line_gap = 20

#--------------------------------------------------------------------------------------------------

#--------------------------------------------region of interest -----------------------------------


trap_bottom_width = 1
trap_top_width = 0.9
trap_height = 0.4


#--------------------------------------------------------------------------------------------------
#lane_lines = []
x = False ; 
bridge=CvBridge() ; 
ang_cl = Point()
ang_cl.x=0.0
ang_cl.y=0.0
ang_cl.z=0.0
s1_old = None
x_offset_old = 1.0
Y_offset_old = 1.0
s1_time = 0.0
pub = None
pub1 = None
pub2 = None

#-----------------------------------------------filtre alpha beta---------------------------------------------
alpha = 0.75
beta = 0.001
prediction = 0.0
estimate = 0.0
sample_time = 0.033
rate = 1




#--------------------------------------------------------------------------------------------------


#----------------------------------------------------------------------------------------
# definition des foncitons
#--------------------------------------------------------------------------------------







'''
def update_fonc() :
    global theta 
    theta.append(ang_cl.x)
    ax.cla()
    ax.plot(theta)
    ax.scatter(len(theta)-1, theta[-1])
    ax.text(len(theta)-1, theta[-1]+2, "{}%".theta(theta[-1]))
    ax.set_ylim(0,100)
''' 


'''
def point(capteur):
    #capteur 1 et capteur 2
    s1=len(capteur)-1
    s2=len(capteur)-1

    for i in range(len(capteur)):
        if capteur[i]!=0:
            s1=i
            break

    if s1!=len(capteur)-1:
        for i in range(len(capteur)-1, s1-1,-1):
            if capteur[i]!=0:
                s2 = i 
                break
        return int((s1+s2)/2)
    return -1
'''

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
 #----------------------------------------------------------------------------------------------------------------------------
    if len(average_line) == 0 and x == False:
       
       s1_time= time.time()
       x= True 
    elif  len(average_line) != 0 :
        s1_time = 0.0
        x = False ; 
        

    global t1
    t1 =time.time()-s1_time
   
    if t1 < 2 and len(average_line) == 0  :
        x_offset = Y_offset_old
        y_offset = Y_offset_old
        error_manager.data = False
        print("temps correct........continuation de la conduite")
        pub.publish(error_manager)
        t1 =time.time()-s1_time

    if   t1 > 2 and len(average_line) == 0 :
       error_manager.data = True
       print("ligne(s) disparue(s)...... amorcage de l'arret d'urgence")
       pub.publish(error_manager)
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
    global s1_old 
    s1_old= lane_lines
    left_fit= []
    right_fit = []

    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
      #   if x1 ==x2:
      #      slopet = 999.
        slopet = (y2-y1)/(x2-x1)
            #------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!checker reaction du filtrage des lignes
        #if  abs(slopet) < 800 :                       
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

    
    if len(lane_lines) !=0 :
        error_manager.data = False
        pub.publish(error_manager)

    return lane_lines



def canny(image):
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    canny_img=cv2.Canny(blur,220,150)
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


##=======================================================================================

s1_old = 0
s2_old = 0
s1 = 0
t1 = 0.0
t2 = 0.0
s2 = 0

s2_time = 0.0
th1 = 75
th2 = 150
stop =0
k= 1 

# ========================================================================================    


def callback(image):
    global ang_cl
    global thetaf
    global Time
    #ang_cl.y est la valeur precedente
    ang_cl.y = ang_cl.x
    try :
        img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)


    #prendre les colonne, ligne et channel
    (ligne,colonne,channels) = img.shape
    print (ligne)
    print (colonne)
    #cv2.imshow('try',img)
    key = cv2.waitKey(10) & 0xFF
    if key == ord("q"):
        raise Exception("on sort")

    # faire copy de l'image
    img_clone = img.copy()

   #-------------------newcod------------------------------------------ 

    canny_image = canny(img_clone)
    cropped_image = region_of_interest(canny_image)
    
    lines = cv2.HoughLinesP(cropped_image,rho,theta,threshold,np.array([]),minLineLength=min_line_length
    ,maxLineGap=max_line_gap)
    averaged_lines = average_slope_intercept(img_clone,lines)
    


    line_image = display_lines(img_clone,averaged_lines)
    
    combo_image = cv2.addWeighted(img_clone,0.8,line_image,1,1)
    angle = give_angle(combo_image, averaged_lines)
    #ang_cl.x est la nouvelle valeur
    anglep = angle-90
    anglepp = anglep / 180.0 * math.pi
    ang_cl.x = anglepp
    print(anglepp)
    #T1 = time.time() - Time
   # thetaf.append((T1 , anglepp))
    

    #update_fonc()
    #plt.show()
    #ang_cl.z est le nombre de ligne
    ang_cl.z = len(averaged_lines)
    #publier les informations
    pub1.publish(ang_cl)
    final_image= display_heading_line(combo_image , angle)
    cv2.imshow('Afficher camera',final_image)
   #---------------------fin code--------------------------------------- 

'''
    #transformer l'imagae en gris pour capteur 1
    gris = cv2.cvtColor(img_clone[ymin:ymax, xmin1:xmax1],cv2.COLOR_BGR2GRAY)
    if k!=1:
        gris = cv2.blur(gris,(k,k))
    
    #appliquer canny
    capteur1 = cv2.Canny(gris,th1,th2)

    #transformer l'imagae en gris pour capteur 2
    gris2 = cv2.cvtColor(img_clone[ymin:ymax, xmin2:xmax2],cv2.COLOR_BGR2GRAY)
    if k!=1:
        gris2 = cv2.blur(gris2,(k,k))
    
    #appliquer canny
    capteur2 = cv2.Canny(gris2,th1,th2)
   
    cv2.rectangle(img_clone,(xmin1,ymin),(xmax1,ymax),(0,0,255),1)
    cv2.rectangle(img_clone,(xmin2,ymin),(xmax2,ymax),(0,0,255),1)

    s1 = point(capteur1[0])
    if s1!=-1:
        cv2.circle(img_clone,(s1+xmin1,ymin),3,(0,255,0), 3)
        s1_old =s1
        global s1_time 
        s1_time= time.time()
    else :
        global t1
        t1 =time.time()-s1_time
        if t1 < 4:
            cv2.circle(img_clone,(s1_old+xmin1,ymin),3,(100,255,255),3)
            s1=s1_old
        else :
            s1=-1

    s2 = point(capteur2[0])
    if s2!=-1:
        cv2.circle(img_clone,(s2+xmin2,ymin),3,(0,255,0), 3)
        s2_old =s2
        global s2_time 
        s2_time = time.time()
    else :
        t2 = time.time()-s2_time 
        if t2< 4:
            cv2.circle(img_clone,(s2_old+xmin2,ymin),3,(100,255,255),3)
            s2=s2_old
        else :
            s2=-1

    cv2.circle(img_clone,(160,120),3,(10,10,10),3)

    cv2.circle(img_clone,(abs((s2+xmin2-s1)/2),ymin),3,(100,100,255),3)

    if s1!=-1 and s2!=-1:
        error_manager.data = False
        pub.publish(error_manager)
        s2_= abs(xmax2-xmin2-s2)
        if abs(s2_-s1) > 20 :
            c = (0,max(0,255-10*int(abs(s1-s2_)/2)),min(255,10*int(abs(s1-s2_)/2)))
            cv2.arrowedLine(img_clone, (int((xmax2-xmin1)/2)+xmin1,ymax-25),(int((xmax2-xmin1)/2)+xmin1+2*int((s1-s2_)/2),ymax-25),c,3,tipLength=0.4)
        else :
            cv2.putText(img_clone,"ok",(int((xmax2-xmin1)/2)+xmin1-15,ymax-16),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,(0,255,0),1)
    elif (s1==-1 or s2==-1) and (t1 > 15 or t2  >= 15) :
        
'''
   
        

def main(args):
 #   if __name__ == '__main__':
    global pub
    global pub1
    global pub2
    global thetaf


 #  ax.set_facecolor('#DEDEDE')
    rospy.init_node('traitement')

    pub = rospy.Publisher("/traitement/error_manager",Bool,queue_size=10)
    
    pub2 = rospy.Publisher("/traitement/filtre",Float64,queue_size=10)

    pub1 = rospy.Publisher("/control/angle",Point,queue_size=10)

    image = rospy.Subscriber("/raspicam_node/image", Image, callback)
   
    rospy.spin()


#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



if __name__ == '__main__':
    main(sys.argv)

   

