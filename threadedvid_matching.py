import cv2 as cv
import numpy as np
from time import time
import datetime
from threading import Thread
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse

class FPS:
	def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
		self._start = None
		self._end = None
		self._numFrames = 0
	def start(self):
		# start the timer
		self._start = datetime.datetime.now()
		return self
	def stop(self):
		# stop the timer
		self._end = datetime.datetime.now()
	def update(self):
		# increment the total number of frames examined during the
		# start and end intervals
		self._numFrames += 1
	def elapsed(self):
		# return the total number of seconds between the start and
		# end interval
		return (self._end - self._start).total_seconds()
	def fps(self):
		# compute the (approximate) frames per second
		return self._numFrames / self.elapsed()
	
class WebcamVideoStream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False


	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self
	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()
	def read(self):
		# return the frame most recently read
		return self.frame
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


vid = WebcamVideoStream(src='game_footage.mkv').start()
fps = FPS().start()
#vid = cv.VideoCapture('game_footage.mkv')
template = cv.imread('balloon.jpg', cv.IMREAD_GRAYSCALE)

#assert template is not None, "file could not be read, check with os.path.exists()"
img2 = template.copy()

cv.threshold(template, 254, 255, cv.THRESH_BINARY)
dilatedImg = cv.dilate(template, (7,7), iterations=3)
cv.erode(template, (5,5), iterations=2)
w, h = template.shape[::-1]

bgHistory = 500
bgVarThreshold = 200
bgDetectShadow = False
obj_dectection = cv.createBackgroundSubtractorMOG2(bgHistory, bgVarThreshold, bgDetectShadow)

while(True):
#while fps._numFrames < 500:
	

	# Capture the video frame
	frame = vid.read()

	roi_frame = frame[ 0:715, 0:1920 ]

	mask = obj_dectection.apply(roi_frame)
	_, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

	dilatedImg = cv.dilate(mask, (7,7), iterations=3)
	erodedImg = cv.erode(dilatedImg, (5,5), iterations=2)
	method = cv.TM_CCOEFF

	contours, hierarchy = cv.findContours(erodedImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

	for count in contours:
		if cv.contourArea(count) > 5000:
			res = cv.matchTemplate(erodedImg,template,method)
			min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
			top_left = max_loc
			bottom_right = (top_left[0] + w, top_left[1] + h)
			cv.rectangle(roi_frame,top_left, bottom_right, 255, 2)

	cv.imshow('frame roi', roi_frame)
	#cv.imshow('eroded', erodedImg)
	#loop_time = time()	
	#print('FPS {}', format(1 / (time() - loop_time)))
	fps.update()
	
	# set 'q' as quit
	if cv.waitKey(1) & 0xFF == ord('q'):
		fps.stop()
		print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
		# After the loop release the cap object
		#vid.release()
		# Destroy all the windows
		cv.destroyAllWindows()
		vid.stop()
		break

print('Done')
