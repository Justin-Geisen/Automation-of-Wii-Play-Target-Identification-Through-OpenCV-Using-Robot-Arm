import cv2 as cv
import numpy as np

templates = []
templates_shape = []
threshold = 0.24

import os
 
# Get the list of all files and directories
path = 'snipped ducks'
dir_list = os.listdir(path)

for img_path in dir_list:
    # print(img_path)
    refPathImg = '{path}/{img_path}'.format(path=path, img_path=img_path)
    # print(refPathImg)
    # print(type(cv.imread(refPathImg)))
    templates.append(cv.imread(refPathImg, cv.IMREAD_GRAYSCALE))
    # templates_shape.append(cv.imread(refPathImg).shape[:: -1])
    # print(type(templates.index))
    # print(type(templates_shape.index))


def doTemplateMatch(gray_img):
    for template in templates: 
        # print(template)
        res = cv.matchTemplate(gray_img, template, cv.TM_CCOEFF_NORMED)
        loc = np.where( res >= threshold )

        w, h = template.shape[::-1]
        # w, h = templates_shape.at[templates.index(template)]
        for pt in zip(*loc[::-1]):
            cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

        cv.imshow('res',res)
        cv.waitKey()

# print("Files and directories in '", path, "' :") 
# prints all files
# print(dir_list)

img_rgb = cv.imread('duckhuntjs_ss\duck_hunt_scene04.png')
img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

doTemplateMatch(img_gray)

# this is to show the edges for the objects
blurImg = cv.GaussianBlur(img_gray, (5,5), cv.BORDER_DEFAULT)
cannyImg = cv.Canny(blurImg, 125, 175)
# cv.imshow("out", cannyImg)
# cv.waitKey()

cv.imshow("out", img_rgb)
cv.waitKey()


# template = cv.imread('snipped ducks/blue5.PNG', cv.IMREAD_GRAYSCALE)
# template = cv.imread('snipped ducks/blue5.PNG', 0)

# w, h = template.shape[::-1]

# res = cv.matchTemplate(img_gray, template, cv.TM_CCOEFF_NORMED)

# loc = np.where( res >= threshold)

# cv.imshow('screen', res)
# cv.imshow('gray', img_gray)
# cv.imshow('template', template)

cv.waitKey(0)