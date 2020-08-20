import cv2

vc=cv2.VideoCapture(0)
if vc.isOpened():
    rval,frame=vc.read()
    while rval:
        rval,frame=vc.read() 
        cv2.imshow("img",frame)
        if cv2.waitKey(10)==ord("1"):
	    break
vc.release()
cv2.destroyAllWindows()       
    
