import cv2 as cv
import numpy as np
import math
import time
import itertools as it

def dist(x,y):
    return math.hypot(y[0]-x[0],y[1]-x[1])

def drawLine( detectedContours, index = 0 ):
	if not detectedContours or len(detectedContours) == 1:
		return None
	
	next_idx = index + 1
	try:
		if detectedContours[next_idx]:
			cv.line(frame, detectedContours[index], detectedContours[next_idx], (255,255, 0),3)
			return drawLine(detectedContours, index + 1)
	except:
		return None
	
def drawLinex( detectedContours, index = 0 ):
	if not detectedContours or len(detectedContours) == 1:
		return None
	
	next_idx = index + 1
	try:
		if detectedContours[next_idx]:
			cv.line(frame, detectedContours[index], detectedContours[next_idx], (0,0, 255),4)
			return drawLine(detectedContours, index + 1)
	except:
		return None

# capture live footage
# vid = cv.VideoCapture( 1 )
vid = cv.VideoCapture('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/game_footage.mkv')

# background subtraction parameter (change to get better detection)
bgHistory = 110
bgVarThreshold = 300
bgDetectShadow = False
obj_dectection = cv.createBackgroundSubtractorMOG2(bgHistory, bgVarThreshold, bgDetectShadow)

# fps counter
prev_frame_time = 0
new_frame_time = 0

while(True):
	
	# Capture the video frame
	ret, frame = vid.read()
	
	# This is the area that we are interested in using
	roi_frame = frame[ 0:715, 0:1920 ]

	mask = obj_dectection.apply(roi_frame)
	# mask = cv.normalize(mask, mask, 0,255, cv.NORM_MINMAX)

	_, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

	dilatedImg = cv.dilate(mask, (69,69), iterations=2)
	erodedImg = cv.erode(dilatedImg, (121,121), iterations=10)
	dilatedImg = cv.dilate(erodedImg, (5,5), iterations=5)

	contours, hierarchy = cv.findContours(dilatedImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

	detectedContours = []

	for count in contours: 
		if cv.contourArea(count) > 3000:
			x, y, w, h = cv.boundingRect(count)
			topLeft = (x, y)
			bottomRight = ( x + w, y + h)
			center = ( (topLeft[0]+bottomRight[0])/2, (topLeft[1]+bottomRight[1])/2 )
			cv.rectangle(frame, topLeft, bottomRight, (255,0,0), 3)
			cv.drawMarker(frame, (int(center[0]),int(center[1])), (255,0,0))
			detectedContours.append((int(center[0]),int(center[1])))
			drawLinex(detectedContours)
			
	# print("----0----",detectedContours)
	
	# nearest point random
	# cur = 0
	# path = [cur]
	# totalDist = 0
	# for i in range(1,len(detectedContours)):
	# 	dists = [(dist(detectedContours[i],p), pi) for (pi,p) in enumerate(detectedContours) if pi != i]
	# 	nextDist, cur = min(dists)
	# 	totalDist += nextDist
	# 	path.append(detectedContours[i])

	# print("----1----",path)


	# sorted shortest path
	paths = [ p for p in it.permutations(detectedContours) ]
	path_distances = [ sum(map(lambda x: dist(x[0],x[1]),zip(p[:-1],p[1:]))) for p in paths ]
	min_index = np.argmin(path_distances)
	print("----2----",paths[min_index])
	
	# draw connecting line between objects	
	drawLine(paths[min_index])

	# fps counter
	new_frame_time = time.time()
	fps = 1/(new_frame_time-prev_frame_time)
	prev_frame_time = new_frame_time
	fps = str(int(fps))
	
	cv.putText(frame, fps, (7, 70), cv.FONT_HERSHEY_COMPLEX_SMALL, 3, (0, 0, 255), 3, cv.LINE_AA)

	# Display the resulting frame
	cv.imshow('frame', frame)
	# cv.imshow('frame roi', roi_frame)
	# cv.imshow('bitWiseOp', bitwiseOp)
	# cv.imshow('fg mask', mask)
	# cv.imshow('dialate', dilatedImg)
	# cv.imshow('eroded', erodedImg)
	
	# set 'q' as quit
	if cv.waitKey(1) & 0xFF == ord('q'):
		break


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()