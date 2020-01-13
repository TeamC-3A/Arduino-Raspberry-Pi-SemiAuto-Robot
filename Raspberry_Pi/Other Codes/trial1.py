import cv2
import numpy as np
import imutils


lower_green = np.array([53,77,33])
upper_green = np.array([88,255,190])

lower_blue = np.array([104,107,104])
upper_blue = np.array([141,255,255])

lower_yellow = np.array([21,91,102])
upper_yellow = np.array([53,202,221])

#######CHECK COLORS##############
def checkColor(frame_f1):


    hsv = cv2.cvtColor(frame_f1, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)


    if(cv2.countNonZero(mask_green)>500):
        return mask_green
        # return "green"

    if(cv2.countNonZero(mask_blue)>500):
        return mask_blue
        # return "blue"
    if(cv2.countNonZero(mask_yellow)>500):
        return mask_yellow
        # return "yellow"
    else:
        return 0


#########DRAW BOXES###############


def drawBoxes(frame, mask):
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # thresh = cv2.threshold(mask,128,255,cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            cntarea = cv2.contourArea(c)

            if (cntarea > 1500):
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(res, (cx, cy), 1, (255, 255, 255), -1)
                cv2.drawContours(res, [c], -1, (0, 255, 0), 3)
                print(cx)
            # print(c)
            # maxcnt = cntarea
            # cv2.drawContours(res, [c], -1, (0, 255, 0), 2)

            # print(cntarea)

        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)
        # cv2.imshow('morph',open_morph)



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

maxcnt = 0
kernal = np.ones((3,3),np.uint8)
while True:

    _,frame = cap.read()
    frame=cv2.resize(frame,None, fx=0.7,fy=0.7,interpolation=cv2.INTER_AREA)
    # frame = cv2.medianBlur(frame,8)
    frame = cv2.GaussianBlur(frame,(15,15),0)
    frame = cv2.flip(frame,0)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    

    # if(checkColor(frame)=="green"):
    #     #     print("green")
    #     # elif(checkColor(frame)=="blue"):
    #     #     print("blue")
    #     # elif(checkColor(frame)=="yellow"):
    #     #     print("yellow")


    try:
        drawBoxes(frame,checkColor(frame))
    except:
        pass

    k = cv2.waitKey(5) & 0xFF
    if k ==27:
        break
    elif  k ==ord('a'):
        print(a)
        a = a+1;
    elif k == ord('b'):
        print(b)
        b= b + 1;
    elif k == ord('v'):
        print(b)
        b = b - 1;
    elif k == ord('t'):
        print(thres)
        thres = thres-1;


cv2.destroyAllWindows()
cap.release()



