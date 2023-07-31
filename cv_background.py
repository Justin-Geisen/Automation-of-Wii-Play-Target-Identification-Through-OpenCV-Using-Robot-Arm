import cv2 as cv
import numpy as np
import math
import time
import itertools as it


def findCursorTM(frame, template):
	grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	grayTemplate = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
	templateW, templateH = grayTemplate.shape[::-1]
	templateRes = cv.matchTemplate(grayFrame, grayTemplate, cv.TM_CCOEFF_NORMED)

	_, maxVal, _, max_loc = cv.minMaxLoc(templateRes)
	# print(maxVal)
	if maxVal >= .7:
		return True, (max_loc[0], max_loc[1], templateW, templateH)
	else:
		return False, (0,0,0,0)
	# return foundLoc[0], foundLoc[1], templateW, templateH
	# return pt, (pt[0] + templateW, pt[1] + templateH)


def detectObj (frame):
	roi_frame = frame[ 0:715, 0:1920 ]

	mask = obj_dectection.apply(roi_frame)
	_, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

	dilatedImg = cv.dilate(mask, (69,69), iterations=2)
	erodedImg = cv.erode(dilatedImg, (121,121), iterations=10)
	dilatedImg = cv.dilate(erodedImg, (5,5), iterations=5)

	contours, _ = cv.findContours(dilatedImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

	detectedContours = []

	for count in contours: 
		cv.drawContours(frame, [count], -1, (0,0,255))
		if cv.contourArea(count) > 3000:
			x, y, w, h = cv.boundingRect(count)
			topLeft = (x, y)
			bottomRight = ( x + w, y + h)
			center = ( (topLeft[0]+bottomRight[0])/2, (topLeft[1]+bottomRight[1])/2 )

			cv.rectangle(frame, topLeft, bottomRight, (255,0,0), 3)
			cv.drawMarker(frame, (int(center[0]),int(center[1])), (255,0,0))
			detectedContours.append((int(center[0]),int(center[1]), int(w/2)))
	
	return detectedContours

def drawLine( detectedContours, index = 0, color = (0,0,0) ):
	if not detectedContours or len(detectedContours) == 1:
		return None
	
	next_idx = index + 1
	try:
		if detectedContours[next_idx]:
			currPos = (detectedContours[index][0], detectedContours[index][1])
			nextPos = (detectedContours[next_idx][0], detectedContours[next_idx][1])
			cv.line(frame, currPos, nextPos, color, 2)
			return drawLine(detectedContours, index + 1, color)
	except:
		return None
	
def dist(x,y):
    return math.hypot(y[0]-x[0],y[1]-x[1])

def sortedShortestPath(detectedContours):
	paths = [ p for p in it.permutations(detectedContours) ]
	path_distances = [ sum(map(lambda x: dist(x[0],x[1]), zip(p[:-1],p[1:]))) for p in paths ]
	min_index = np.argmin(path_distances)
	return paths[min_index]

# capture live footage
# vid = cv.VideoCapture( 1 )

###### MAIN LOOP ######

# vid = cv.VideoCapture('Video\game_footage.mkv')
# vid = cv.VideoCapture('Video\game_footage_2.mkv')
vid = cv.VideoCapture('Video\game_footage_3.mkv')
template = cv.imread('cursor.png')

# background subtraction parameter (change to get better detection)
bgHistory = 110
bgVarThreshold = 300
bgDetectShadow = False
obj_dectection = cv.createBackgroundSubtractorMOG2(bgHistory, bgVarThreshold, bgDetectShadow)

# tracker = cv.legacy.TrackerMOSSE_create()
tracker = cv.TrackerCSRT_create()
# tracker = cv.TrackerKCF_create()

# fps counter
prev_frame_time = 0
new_frame_time = 0
frame_count = 0

isCursorFound = False

while(True):
	
	# Capture the video frameq
	ret, frame = vid.read()

	returnSuccc = False
	# find the Cursor
	if not isCursorFound:
		cursorReturn, cursorRectangle = findCursorTM(frame, template)
		if cursorReturn :
			tracker.init(frame, cursorRectangle)
			isCursorFound = True
	else: 
		returnSuccc, cursorBox = tracker.update(frame)

	if returnSuccc:
		cBoxTL = (int(cursorBox[0]), int(cursorBox[1]))
		cBoxBR = (int(cursorBox[0] + cursorBox[2]), int(cursorBox[1] + cursorBox[3]))
		cBoxCenter = (int((cBoxTL[0] +cBoxBR[0])/2), int((cBoxTL[1] + cBoxBR[1])/2))
		cv.rectangle(frame, cBoxTL, cBoxBR, (255,0,0), 2, 1)
		cv.drawMarker(frame, (cBoxCenter[0], cBoxCenter[1]), (255,0,0))
		cv.putText(frame, "Tracking cursor", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
	else:
		cv.putText(frame, "Tracking cursor failure", (100,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

	# TODO: broken code, need to find solutions
	# if isCursorFound: 
	# 	# This is the area that we are interested in using
	# 	detectedContours = detectObj(frame)

	# 	# This sorts the list of detected contours
	# 	sortedDetectedContours = sortedShortestPath(detectedContours)

	# 	# print(detectedContours)
	# 	drawLine(sortedDetectedContours, color=(255,255, 0))

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
	
	frame_count = frame_count + 1
	# set 'q' as quit
	if cv.waitKey(1) & 0xFF == ord('q'):
		break


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()