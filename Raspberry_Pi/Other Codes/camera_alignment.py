import cv2



cap= cv2.VideoCapture(0)


while True:
    _,frame = cap.read()
    #frame = cv2.flip(frame,0)
    
    slicedImage = frame[320:475, 434:640]
    cv2.imshow("sliced", slicedImage)
    
    frame[320:475, 434:640] = (0,0,0)
    
    
    cv2.imshow("cam",frame)
    
    
    
    
    k=cv2.waitKey(5)
    if k==27:
        break
    
cv2.destroyAllWindows()
cap.release()