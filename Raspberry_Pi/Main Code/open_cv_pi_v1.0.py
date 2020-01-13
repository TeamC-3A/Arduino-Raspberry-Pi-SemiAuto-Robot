import cv2
import numpy as np
import imutils
import serial


import RPi.GPIO as GPIO
from time import sleep

port = serial.Serial("/dev/serial0", baudrate=38400, timeout=3.0)
pinout = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pinout, GPIO.OUT)
pi_pwm = GPIO.PWM(pinout,1000)
pi_pwm.start(0)
prev_xcor=0;
showimage =True;
sampleCount = 0;



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
cv2.namedWindow("cropper")

def map1(x, in_min, in_max, out_min, out_max):

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min




with open('blue_constants.txt','r') as fg:
    lines = fg.readlines()
    line3 = lines[2]
    line4 = lines[3]

    l_h_val = int(line3.split(',')[0])
    l_s_val = int(line3.split(',')[1])
    l_v_val = int(line3.split(',')[2])

    u_h_val = int(line4.split(',')[0])
    u_s_val = int(line4.split(',')[1])
    u_v_val = int(line4.split(',')[2])


    print(l_v_val )


with open('yellow_constants.txt','r') as fg:
    lines = fg.readlines()
    line3 = lines[2]
    line4 = lines[3]

    l_h_val_y = int(line3.split(',')[0])
    l_s_val_y = int(line3.split(',')[1])
    l_v_val_y = int(line3.split(',')[2])

    u_h_val_y = int(line4.split(',')[0])
    u_s_val_y = int(line4.split(',')[1])
    u_v_val_y = int(line4.split(',')[2])








with open('green_constants.txt','r') as fg:
    lines = fg.readlines()
    line3 = lines[2]
    line4 = lines[3]

    l_h_val_g = int(line3.split(',')[0])
    l_s_val_g = int(line3.split(',')[1])
    l_v_val_g = int(line3.split(',')[2])

    u_h_val_g = int(line4.split(',')[0])
    u_s_val_g = int(line4.split(',')[1])
    u_v_val_g = int(line4.split(',')[2])



with open('crop_constants.txt','r') as fg:
    lines = fg.readlines()
    line1 = lines[0]
    # line4 = lines[3]

    y_crop_cal = line1.split(',')[0]
    x_crop_cal = line1.split(',')[1]


    print(l_v_val )



cv2.createTrackbar("l -h","trackbases",int(l_h_val),255,nothing)
cv2.createTrackbar("l -s","trackbases",int(l_s_val),255,nothing)
cv2.createTrackbar("l -v","trackbases",int(l_v_val),255,nothing)
cv2.createTrackbar("u -h","trackbases",int(u_h_val),255,nothing)
cv2.createTrackbar("u -s","trackbases",int(u_s_val),255,nothing)
cv2.createTrackbar("u -v","trackbases",int(u_v_val),255,nothing)

cv2.createTrackbar("y", "cropper",int(y_crop_cal),300,nothing)
cv2.createTrackbar("x", "cropper",int(x_crop_cal),300,nothing)



def trackers(hsv):

    return mask



maxcnt = 0
kernal = np.ones((3,3),np.uint8)


def green():
    global sampleCount
    sampleCount+=1
    port.write(bytes('g', 'utf-8'))
    return "green"

def blue():
    global sampleCount
    sampleCount+=1
    port.write(bytes('b', 'utf-8'))
    return "blue"
    

def yellow():
    global sampleCount
    sampleCount+=1
    port.write(bytes('y', 'utf-8'))
    return "yellow"

class Color:
    colname_lower=0
    colname_higher=0

color = Color()

def checkColor(crop_segment):
    
    
    hsv1 = cv2.cvtColor(crop_segment,cv2.COLOR_BGR2HSV)

    val_blue_low = np.array([l_h, l_s, l_v])
    val_blue_high = np.array([u_h, u_s, u_v])

    val_yellow_low = np.array([l_h_val_y, l_s_val_y, l_v_val_y])
    val_yellow_high = np.array([u_h_val_y, u_s_val_y, u_v_val_y])

    val_green_low = np.array([l_h_val_g, l_s_val_g, l_v_val_g])
    val_green_high = np.array([u_h_val_g, u_s_val_g, u_v_val_g])


    lower_blue = val_blue_low
    upper_blue = val_blue_high

    lower_yellow = val_yellow_low
    upper_yellow = val_yellow_high

    lower_green = val_green_low
    upper_green = val_green_high


    mask_blue = cv2.inRange(hsv1,lower_blue,upper_blue)
    mask_yellow = cv2.inRange(hsv1, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv1, lower_green, upper_green)


    blue_count = cv2.countNonZero(mask_blue)
    yellow_count =cv2.countNonZero(mask_yellow)
    green_count =cv2.countNonZero(mask_green)

    # print("blue: {}".format(blue_count))
    arr = np.array([500,green_count,blue_count,yellow_count])
    max_index = np.where(arr==np.amax(arr))[0][0]
    global sampleCount
    

    if(max_index==0):
        return "nocolor"
    elif(max_index==1):
        sampleCount+=1
        color.colname_lower = lower_green
        color.colname_higher = upper_green
        port.write(bytes('g', 'utf-8'))
    elif(max_index==2):
        sampleCount+=1
        color.colname_lower = lower_blue
        color.colname_higher = upper_blue
        port.write(bytes('b', 'utf-8'))

    elif(max_index==3):
        sampleCount+=1
        color.colname_lower = lower_yellow
        color.colname_higher = upper_yellow
        port.write(bytes('y', 'utf-8'))
        
    else:
        return "nocolor"
    
   
       
        
    
    
    
#     print(sampleCount)
    












cap = cv2.VideoCapture(0)

# cap = cv2.VideoCapture("D:\\Amila\\New folder\\a.mp4")
while 1:
    rec_data = port.readline(1)
    
    print(rec_data)
    if(rec_data==b'y' or rec_data==b'\xe5'):
        break


while sampleCount<160:
    _, frame = cap.read()
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    y_crop = cv2.getTrackbarPos("y", "cropper")
    x_crop = cv2.getTrackbarPos("x", "cropper")
    
    l_h = cv2.getTrackbarPos("l -h", "trackbases")
    l_s = cv2.getTrackbarPos("l -s", "trackbases")
    l_v = cv2.getTrackbarPos("l -v", "trackbases")
    u_h = cv2.getTrackbarPos("u -h", "trackbases")
    u_s = cv2.getTrackbarPos("u -s", "trackbases")
    u_v = cv2.getTrackbarPos("u -v", "trackbases")

    crop_segment = frame[y_crop:640, x_crop:300]
    checkColor(crop_segment)
    cv2.imshow("frame", frame)
    
    cv2.imshow("cropped", crop_segment)
    print(sampleCount)

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break


cv2.destroyAllWindows()

cv2.namedWindow("trackbases")
cv2.namedWindow("cropper")

cv2.createTrackbar("l -h","trackbases",int(l_h_val),255,nothing)
cv2.createTrackbar("l -s","trackbases",int(l_s_val),255,nothing)
cv2.createTrackbar("l -v","trackbases",int(l_v_val),255,nothing)
cv2.createTrackbar("u -h","trackbases",int(u_h_val),255,nothing)
cv2.createTrackbar("u -s","trackbases",int(u_s_val),255,nothing)
cv2.createTrackbar("u -v","trackbases",int(u_v_val),255,nothing)

cv2.createTrackbar("y", "cropper",int(y_crop_cal),300,nothing)
cv2.createTrackbar("x", "cropper",int(x_crop_cal),300,nothing)








while True:

    _,frame = cap.read()
    frame=cv2.resize(frame,None, fx=0.5,fy=0.5,interpolation=cv2.INTER_AREA)
    # frame = cv2.medianBlur(frame,8)
    #frame=cv2.flip(frame,0)
    




    y_crop = cv2.getTrackbarPos("y", "cropper")
    x_crop = cv2.getTrackbarPos("x", "cropper")

    crop_segment = frame[y_crop:640, x_crop:300]

    


    frame = cv2.GaussianBlur(frame,(5,5),0)
    frame[y_crop:640, x_crop:320] = (0,0,0)
    kernal = np.ones((3, 3), np.uint8)
    cv2.imshow('cropped',crop_segment)

    # frame = cv2.line(frame, (int(width/4),0), (int(width/4),int(height)), (0,0,0), 50)
    # frame = cv2.line(frame, (int(3*(width/4)),0), (int(3*(width/4)),int(height)), (0,0,0), 50)



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
    lower_red = color.colname_lower
    upper_red=  color.colname_higher

    mask = cv2.inRange(hsv,lower_red,upper_red)
    
    if(cv2.countNonZero(mask)<1000):
#      port.write(bytes('s', 'utf-8'))
        
     print(cv2.countNonZero(mask))



#     open_morph = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernal,iterations=2)
# 
#     total_white = cv2.countNonZero(mask);
#     print(total_white)

    res = cv2.bitwise_and(frame,frame,mask=mask)
   # thresh = cv2.threshold(mask,128,255,cv2.THRESH_BINARY)[1]



    cnts = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if(np.array(cnts).size>0):
        maxcnt = max(cnts,key=cv2.contourArea)




        
        area = cv2.contourArea(maxcnt)
            

        if(area>500):
#                 x, y, w, h = cv2.boundingRect(maxcnt)
#                 cv2.rectangle(_, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # cv2.drawContours(res, [maxcnt], -1, (0, 255, 0), 3, -1)
            M = cv2.moments(maxcnt)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            cv2.circle(res, (cx, cy), 1, (255, 255, 255), -1)
#                 cx=int(x+(w/2))
#                 cy=int(y+(h/2))
            x_cor = map1(cx, 0, 300, 0, 9)
            str1 = str(round(x_cor))
            port.write(bytes(str1, 'utf-8'))
#                 
            cv2.circle(res, (cx,cy), 2, (255, 255, 255), -1)
        else:
            port.write(bytes('s', 'utf-8'))



        def showImage():        
            cv2.imshow('frame',frame)
            cv2.imshow('mask',mask)
            cv2.imshow('res',res)
            
        if(showimage):
            showImage()
#     cv2.imshow('morph',open_morph)

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break

    elif k==ord('b'):
        with open('blue_constants.txt', 'w+') as the_file:
            str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{},{}'.format(l_h,l_s,l_v)
            str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str1+'\n'+str2+'\n'+str3+'\n'+str4)
    elif k==ord('g'):
        with open('green_constants.txt', 'w+') as the_file:
            str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{},{}'.format(l_h,l_s,l_v)
            str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str1+'\n'+str2+'\n'+str3+'\n'+str4)
    elif k==ord('y'):
        with open('yellow_constants.txt', 'w+') as the_file:
            str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{},{}'.format(l_h,l_s,l_v)
            str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str1+'\n'+str2+'\n'+str3+'\n'+str4)
    elif k==ord('c'):
        with open('crop_constants.txt', 'w+') as the_file:
            # str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            # str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{}'.format(y_crop,x_crop)
            # str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str3)
    elif k==ord('r'):
        with open('red_constants.txt', 'w+') as the_file:
            str1='val1 = np.array([{},{},{}])'.format(l_h,l_s,l_v)
            str2='val2 = np.array([{},{},{}])'.format(u_h,u_s,u_v)
            str3 = '{},{},{}'.format(l_h,l_s,l_v)
            str4 = '{},{},{}'.format(u_h,u_s,u_v)
            the_file.write(str1+'\n'+str2+'\n'+str3+'\n'+str4)
    elif k==ord('i'):
        if(showimage==True):
            showimage=False
        else:
            showimage=True
        
cv2.destroyAllWindows()
cap.release()