# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import numpy as np
import struct
import smbus
import time
import picamera
import io
import picamera.array

bus = smbus.SMBus(1)

#
# this is the Slave address of the Arduino
#
address = 0x04

#
# initialize dummy value of output from Pi (bytes only)
#


#
# initialize dummy values of inputs to Pi
#
dummyToPiFloats = [-3.1416, 6.2832]
dummyToPiBytes = [2047, 50, 50]

# construct the argument parse and parse the arguments
##ap = argparse.ArgumentParser()
##ap.add_argument("-v", "--video",
##	help="path to the (optional) video file")
##ap.add_argument("-b", "--buffer", type=int, default=64,
##	help="max buffer size")
##args = vars(ap.parse_args())

# define the lower and upper boundaries of the "orange"
# ball in the HSV coim redalor space, then initialize the
# list of tracked points
lower_limit = np.array([0, 60, 60])
upper_limit = np.array([10, 255, 255])

#lower_limit = (110, 50, 50)
#upper_limit = (130, 255, 255)
##pts = deque(maxlen=args["buffer"])

x = 100
y = 150
vs = cv2.VideoCapture(0)
vs.set(3, 256)
vs.set(4, 144)
#vs.set(3, 354)
#vs.set(4, 240)
stream = io.BytesIO()
# allow the camera or video file to warm up
time.sleep(2.0)

def putByteList(byteList):
    try:
        bus.write_i2c_block_data(address, 255, byteList)
    except:
        print("error writing commands")
    return None

# keep looping
while True:
	# grab the current frame
	ret, frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	#frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	#frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "orange", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, lower_limit, upper_limit)

	# show the frame to our screen
	cv2.imshow("Frame2", mask)

	key = cv2.waitKey(1) & 0xFF
	#mask = cv2.erode(mask, None, iterations=2)
	#mask = cv2.dilate(mask, None, iterations=2)



	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	center = None
	x = int(x)
	y = int(y)
	coordinates = [x,y]
	print(x)
	putByteList(coordinates)

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		if ((M["m10"] != 0) and (M["m10"] != 0) and (M["m10"] != 0)):
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
                        if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
                                cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
                                cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# update the points queue
	#pts.appendleft(center)

	# loop over the set of tracked points
##	for i in range(1, len(pts)):
##		# if either of the tracked points are None, ignore
##		# them
##		if pts[i - 1] is None or pts[i] is None:
##			continue
##
##		# otherwise, compute the thickness of the line and
##		# draw the connecting lines
##		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
##		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()