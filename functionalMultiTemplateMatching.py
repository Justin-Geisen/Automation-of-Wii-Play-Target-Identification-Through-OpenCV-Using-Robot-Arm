import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import os
templates=[]
templates_shape = []
results=[]
resultLoc=[]

path="C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/balloonTemplates"  #May be needed for video?
dir_list=os.listdir(path) #lists all files in path
#print(dir_list)

img_rgb = cv.imread('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/balloonParent.png')
assert img_rgb is not None, "file could not be read, check with os.path.exists()"
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
#wR, hR = templateRed.shape[::-1]
#wB, hB=templateBlue.shape[::-1]
#wY, hY=templateYellow.shape[::-1]

def makeTemplates(dirlist):

    for img in dirlist:
        refPathImg = '{path}/{img}'.format(path=path, img=img)
        templates.append(cv.imread(refPathImg, cv.IMREAD_GRAYSCALE))
        templates_shape.append(cv.imread(refPathImg).shape[::]) #change from -1 to 0
        print(cv.imread(refPathImg).shape[::-1])
        print(type(templates.index))
        print(type(templates_shape.index))

def findResult(templates):
    for template in templates:
       results.append(cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED))

def findLocs(results,threshold):
    for result in results:
        resultLoc.append(np.where( result >= threshold))


def drawSquaresAroundFound(templates_shape,img_rgb,resultLoc): 
    count=0
    for loc in resultLoc:
       for pt in zip(*loc[::-1]):
            cv.rectangle(img_rgb, pt, (pt[0] + templates_shape[count][1], pt[1] + templates_shape[count][0]), (0,165,255), 2)
       count=count+1
        

makeTemplates(dir_list)
findResult(templates)
findLocs(results, .7)
drawSquaresAroundFound(templates_shape,img_rgb,resultLoc)

for x in templates_shape:
    print(x)
#cv.imshow("template0",templates[0])
#cv.imshow("template1",templates[1])
#cv.imshow("template2",templates[2])
cv.imshow('resultYellow',results[2])
cv.imshow('mas',img_rgb)
cv.waitKey(0)


#def doTemplateMatch(gray_img):

