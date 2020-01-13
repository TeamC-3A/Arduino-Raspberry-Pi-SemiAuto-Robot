import cv2
import numpy as np
import imutils
import serial
import jedi 

import RPi.GPIO as GPIO
from time import sleep

port = serial.Serial("/dev/serial0", baudrate=38400, timeout=1.0)
pinout = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pinout, GPIO.OUT)
pi_pwm = GPIO.PWM(pinout,1000)
pi_pwm.start(0)
prev_xcor=0;


cap = cv2.VideoCapture(0)
a = 200
b=100
thres = 10;

bgr_blue = np.uint8([[[180,160,125]]])
hsv1 = cv2.cvtColor(bgr_blue, cv2.COLOR_BGR2HSV)
print(hsv1)
bgr_yellow = np.uint8([[[91,168,187]]])
hsv2 = cv2.cvtColor(bgr_yellow, cv2.COLOR_BGR2HSV)
print(hsv2)
bgr_green = np.uint8([[[91,107,59]]])
hsv3 = cv2.cvtColor(bgr_green, cv2.COLOR_BGR2HSV)
print(hsv3)
def nothing(x):
    pass
cv2.namedWindow("trackbases")

def map1(x, in_min, in_max, out_min, out_max):

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min




with open('blue_constants.txt','r') as fg:
    lines = fg.readlines()
    line3 = lines[2]
    line4 = lines[3]

    l_h_val = line3.split(',')[0]
    l_s_val = line3.split(',')[1]
    l_v_val = line3.split(',')[2]

    u_h_val = line4.split(',')[0]
    u_s_val = line4.split(',')[1]
    u_v_val = line4.split(',')[2]


    print(l_v_val )



cv2.createTrackbar("l -h","trackbases",int(l_h_val),255,nothing)
cv2.createTrackbar("l -s","trackbases",int(l_s_val),255,nothing)
cv2.createTrackbar("l -v","trackbases",int(l_v_val),255,nothing)
cv2.createTrackbar("u -h","trackbases",int(u_h_val),255,nothing)
cv2.createTrackbar("u -s","trackbases",int(u_s_val),255,nothing)
cv2.createTrackbar("u -v","trackbases",int(u_v_val),255,nothing)


def trackers(hsv):

    return mask



maxcnt = 0
kernal = np.ones((3,3),np.uint8)


while True:

    _,frame = cap.read()
    frame=cv2.resize(frame,None, fx=0.5,fy=0.5,interpolation=cv2.INTER_AREA)
    # frame = cv2.medianBlur(frame,8)
    #frame=cv2.flip(frame,0)
    frame = cv2.GaussianBlur(frame,(5,5),0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


    l_h = cv2.getTrackbarPos("l -h", "trackbases")
    l_s = cv2.getTrackbarPos("l -s", "trackbases")
    l_v = cv2.getTrackbarPos("l -v", "trackbases")
    u_h = cv2.getTrackbarPos("u -h", "trackbases")
    u_s = cv2.getTrackbarPos("u -s", "trackbases")
    u_v = cv2.getTrackbarPos("u -v", "trackbases")


    val1 = np.array([l_h,l_s,l_v])
    val2 = np.array([u_h,u_s,u_v])
#     val1 = np.array([55,141,42])
#     val2 = np.array([123,255,255])
    lower_red = val1
    upper_red=  val2

    mask = cv2.inRange(hsv,lower_red,upper_red)
    
    if(cv2.countNonZero(mask)<500):
        port.write(bytes('ss', 'utf-8'))
        
#     print(cv2.countNonZero(mask))



#     open_morph = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernal,iterations=2)
# 
#     total_white = cv2.countNonZero(mask);
#     print(total_white)

    res = cv2.bitwise_and(frame,frame,mask=mask)
   # thresh = cv2.threshold(mask,128,255,cv2.THRESH_BINARY)[1]


    cnts = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    
#     cntarea = cv2.contourArea(c)
#     print(cntarea);
#  
# #      try:
# #          maxcnt = max(cnts,key=cv2.contourArea)
# #      except:
# #         pass
#  
#     if(cntarea>21000):
#         port.write(bytes('ss', 'utf-8'))

#      if(cntarea>=maxcnt ):
#                M = cv2.moments(c)
#                cx = int(M["m10"]/M["m00"])
#                x_cor = map1(cx,0,320,10,99)
#                str1 = str(round(x_cor))
#                port.write(bytes(str1, 'utf-8'))
#                print(str1)
    for c in cnts:
        cntareal = cv2.contourArea(c)
#         print(cntareal)
    
    if(cntareal>600):
        try:
               maxcnt = max(cnts,key=cv2.contourArea)
               M = cv2.moments(maxcnt)
               cx = int(M["m10"]/M["m00"])
               x_cor = map1(cx,0,320,0,9)
               cy = int(M["m01"] / M["m00"])
               cv2.circle(res,(cx,cy),1,(255,255,255),-1)
               cv2.drawContours(res,[maxcnt],-1,(0,255,0),3)
               str1 = str(round(x_cor))
               if(cv2.contourArea(maxcnt)>23000):
                   port.write(bytes('ss', 'utf-8'))
               else:
                   
                   
                   port.write(bytes('\0','utf-8'))
                   port.write(bytes(str1, 'utf-8'))
                   port.write(bytes('\0','utf-8'))
                   
                   print(str1)
        except:
               pass




           
#                if(prev_xcor!=x_cor):
             
               
           
           #print(cx)


      
     #print(c)
    # maxcnt = cntarea
    # cv2.drawContours(res, [c], -1, (0, 255, 0), 2)

 #print(cntarea)







    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
#     cv2.imshow('morph',open_morph)

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break
    elif  k ==ord('a'):
        print(a)
        a = a+1;
    elif k == ord('3'):
        print(b)
        b= b + 1;
    elif k == ord('v'):
        print(b)
        b = b - 1;
    elif k == ord('t'):
        print(thres)
        thres = thres-1;
    elif k==ord('b'):
        with open('blue_constants.txt', 'w+') as the_file:
            str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{},{}'.format(l_h,l_s,l_v)
            str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str1+'\n'+str2+'\n'+str3+'\n'+str4)


cv2.destroyAllWindows()
cap.release()