import cv2
import numpy as np
print("test1")
cam = cv2.VideoCapture(0)
print("test2")
while True:
	ret, image = cam.read()
	cv2.imshow('Imagetest',image)
	k = cv2.waitKey(1)
	if k != -1:
		break
print("test3")
ret, image = cam.read()
print("test4")
cv2.imshow("imgTest",image)
print("test5")
cv2.imwrite('testimage.jpg', image)
print("test6")
cam.release()
cv2.destroyAllWindows()
