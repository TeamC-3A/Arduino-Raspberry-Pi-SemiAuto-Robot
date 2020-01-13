import cv2
import numpy as np
import imutils


cv2.namedWindow("trackbases")
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

def nothing(x):
    pass

cv2.createTrackbar("l -h","trackbases",int(l_h_val),255,nothing)
cv2.createTrackbar("l -s","trackbases",int(l_s_val),255,nothing)
cv2.createTrackbar("l -v","trackbases",int(l_v_val),255,nothing)
cv2.createTrackbar("u -h","trackbases",int(u_h_val),255,nothing)
cv2.createTrackbar("u -s","trackbases",int(u_s_val),255,nothing)
cv2.createTrackbar("u -v","trackbases",int(u_v_val),255,nothing)



while True:

    frame = cv2.imread("D:\\Amila\\New folder\\q (6).jpg")

    frame = cv2.resize(frame,None, fx=0.1,fy=0.1,interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("l -h", "trackbases")
    l_s = cv2.getTrackbarPos("l -s", "trackbases")
    l_v = cv2.getTrackbarPos("l -v", "trackbases")
    u_h = cv2.getTrackbarPos("u -h", "trackbases")
    u_s = cv2.getTrackbarPos("u -s", "trackbases")
    u_v = cv2.getTrackbarPos("u -v", "trackbases")

    val1 = np.array([l_h, l_s, l_v])
    val2 = np.array([u_h, u_s, u_v])
    #     val1 = np.array([55,141,42])
    #     val2 = np.array([123,255,255])
    lower_red = val1
    upper_red = val2

    mask = cv2.inRange(hsv,lower_red,upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('mas',mask)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = imutils.grab_contours(cnts)


    for c in cnts:

        # cv2.drawContours(res, [c], -1, (0, 255, 0), 3, -1)
        # M = cv2.moments(c)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(res, [box], 0, (0, 0, 255), 2)
        # cx = int(M["m10"] / M["m00"])
        # cy = int(M["m01"] / M["m00"])
        #
        # cv2.circle(res, (cx, cy), 1, (255, 255, 255), -1)
        print(cv2.contourArea(c))


    cv2.imshow('res',res)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break;

cv2.destroyAllWindows()
