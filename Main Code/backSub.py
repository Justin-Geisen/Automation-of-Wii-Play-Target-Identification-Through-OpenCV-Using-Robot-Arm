import cv2 as cv
import numpy as np


# capture live footage
# vid = cv.VideoCapture(1)
vid = cv.VideoCapture('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/game_footage.mkv')

bgHistory = 120
bgVarThreshold = 100
bgDetectShadow = False
obj_dectection = cv.createBackgroundSubtractorMOG2(bgHistory, bgVarThreshold, bgDetectShadow)

while(True):
	
	# Capture the video frame
	ret, frame = vid.read()

	roi_frame = frame[ 0:715, 0:1920 ]

	mask = obj_dectection.apply(roi_frame)
	_, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

	dilatedImg = cv.dilate(mask, (7,7), iterations=3)
	erodedImg = cv.erode(dilatedImg, (5,5), iterations=2)

	contours, hierarchy = cv.findContours(erodedImg, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

	outlineMask = np.zeros( (frame.shape[0], frame.shape[1], 3 ), dtype=np.uint8)     

	for count in contours:
		area = cv.contourArea(count) 
		if area > 500:
			# print(area)
			# cv.drawContours(roi_frame, [count], -1, (0,0,255), 2)
			x, y, w, h = cv.boundingRect(count)
			cv.rectangle(outlineMask, (x, y), ( x + w, y + h), (255,255,255), -1)

	bitwise = cv.bitwise_and(frame, outlineMask)

	# Display the resulting frame
	# cv.imshow('frame', frame)
	# cv.imshow('frame roi', roi_frame)
	# cv.imshow('black_box', outlineMask)
	cv.imshow('black_box', bitwise)
	# cv.imshow('bg mask', mask)
	# cv.imshow('dialate', dilatedImg)
	# cv.imshow('eroded', erodedImg)
	
	# set 'q' as quit
	if cv.waitKey(1) & 0xFF == ord('q'):
		break


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv.destroyAllWindows()