import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
img_rgb = cv.imread('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/balloonParent.png')
assert img_rgb is not None, "file could not be read, check with os.path.exists()"
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
templateRed = cv.imread('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/redballoontemplate.png', cv.IMREAD_GRAYSCALE) 
templateBlue=cv.imread('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/blueballoontemplate.png', cv.IMREAD_GRAYSCALE) 
templateYellow=cv.imread('C:/Users/mrssp/Desktop/duckhunt/templateMatchingTest/yellowballoontemplate.png', cv.IMREAD_GRAYSCALE) 
wR, hR = templateRed.shape[::-1]
wB, hB=templateBlue.shape[::-1]
wY, hY=templateYellow.shape[::-1]
resRed=cv.matchTemplate(img_gray,templateRed,cv.TM_CCOEFF_NORMED)
resYellow=cv.matchTemplate(img_gray,templateYellow,cv.TM_CCOEFF_NORMED)
resBlue=cv.matchTemplate(img_gray,templateBlue,cv.TM_CCOEFF_NORMED)

threshold = 0.7
locRed = np.where( resRed >= threshold)
locBlue = np.where( resBlue >= threshold)
locYellow = np.where( resYellow >= threshold)
for pt in zip(*locRed[::-1]):
    #cv.rectangle(img_rgb, pt, (pt[0] + wY, pt[1] + hY), (255,255,0), 2)
    cv.rectangle(img_rgb, pt, (pt[0] + wR, pt[1] + hR), (0,0,255), 2)
     #cv.rectangle(img_rgb, pt, (pt[0] + wB, pt[1] + hB), (255,0,0), 2)

for pt in zip(*locYellow[::-1]):
    cv.rectangle(img_rgb, pt, (pt[0] + wY, pt[1] + hY), (0,255,255), 2)
   # cv.rectangle(img_rgb, pt, (pt[0] + wR, pt[1] + hR), (0,0,255), 2)
     #cv.rectangle(img_rgb, pt, (pt[0] + wB, pt[1] + hB), (255,0,0), 2)     

for pt in zip(*locBlue[::-1]):
    #cv.rectangle(img_rgb, pt, (pt[0] + wY, pt[1] + hY), (255,255,0), 2)
   # cv.rectangle(img_rgb, pt, (pt[0] + wR, pt[1] + hR), (0,0,255), 2)
    cv.rectangle(img_rgb, pt, (pt[0] + wB, pt[1] + hB), (255,0,0), 2)   
cv.imwrite('res.png',img_rgb)
cv.imshow('result',resYellow)
cv.imshow('mas',img_rgb)
cv.imshow('template',templateYellow)
cv.waitKey(0)