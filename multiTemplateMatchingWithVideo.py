import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import os

path="C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/balloonTemplates"  #May be needed for video?
dir_list=os.listdir(path) #lists all files in path
#print(dir_list)

templates=[]
templates_shape = []
results=[]
resultLoc=[]


def makeTemplates(dirlist):

    for img in dirlist:
        refPathImg = '{path}/{img}'.format(path=path, img=img)
        templates.append(cv.imread(refPathImg, cv.IMREAD_GRAYSCALE))
        templates_shape.append(cv.imread(refPathImg).shape[::]) #change from -1 to 0
        #print(cv.imread(refPathImg).shape[::])
        #print(type(templates.index))
        #print(type(templates_shape.index))

def findResult(templates,img_gray):
    for template in templates:
       results.append(cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED))

def findLocs(results,threshold):
    for result in results:
        resultLoc.append(np.where( result >= threshold))
    if len(resultLoc)>3: #changed from four to three cant figure out a way to 
       resultLoc.clear()



def drawSquaresAroundFound(templates_shape,img_rgb,resultLoc): 
    
    count=0
    #print(*templates_shape)
    #print(*resultLoc)
    for loc in resultLoc: #change from loc in result loc?
       for pt in zip(*loc[::-1]):
          #print(count)
          cv.rectangle(img_rgb, pt, (pt[0] + templates_shape[count][1], pt[1] + templates_shape[count][0]), (0,165,255), 2)
            
       count=count+1
    
    


makeTemplates(dir_list)
cap = cv.VideoCapture('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/wiiplayshooting.mp4')
for template in templates_shape:
    print(template)

while cap.isOpened():
 ret, frame = cap.read() #frame can be thought of as like img_rgb
 # if frame is read correctly ret is True
 if not ret:
    print("Can't receive frame (stream end?). Exiting ...") #maybe dont break?
    break
 gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
 frame=cv.resize(frame,(0,0), fx=.2,fy=.2)
 findResult(templates,gray)
 findLocs(results, .7)
 drawSquaresAroundFound(templates_shape,frame,resultLoc)
 #cv.imshow('frame', gray)
 cv.imshow('changedFrame',frame)
 if cv.waitKey(1) == ord('q'):
    break
 

cap.release()
cv.destroyAllWindows()