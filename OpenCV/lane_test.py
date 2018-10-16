import cv2
import numpy as np

imgColor = cv2.imread('blackRoad.jpg') 
#
# 
#
img = cv2.cvtColor(imgColor, cv2.COLOR_BGR2GRAY)
#
# 
img = cv2.GaussianBlur(img,(3,3),0)
#
#
#
edges = cv2.Canny(img,20,80)
#
#
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])

mask_yellow = cv2.inRange(img, lower_yellow, upper_yellow)
mask_white = cv2.inRange(img, 200, 255)
mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)


cv2.imshow('Edges',edges)
cv2.imwrite('edges.jpg',edges)
#

#
minLineLength = 50
maxLineGap = 10
lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=50,minLineLength=minLineLength,maxLineGap=maxLineGap)
#

#
for x in range(0, len(lines)):
        
    for x1,y1,x2,y2 in lines[x]:
        if(y1 > 280 & y2 > 280):
        	cv2.line(imgColor,(x1,y1),(x2,y2),(0,0,255),2) # draw the line on the original image
        	print(x1, y1, x2, y2)



cv2.imshow('hough',imgColor)
cv2.imwrite('hough.jpg',imgColor)
#
# the graphics windows opened by CV2 seem to freak out
# if you don't have this command at the end
#
cv2.waitKey(0)