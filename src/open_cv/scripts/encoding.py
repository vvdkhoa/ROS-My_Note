#!/usr/bin/env python

from turtle import color
import numpy as np
import cv2

# Read img
image_path = "src/open_cv/images/tree.jpg"
img = cv2.imread(image_path, cv2.IMREAD_COLOR)

# Display img in native color
cv2.imshow("Ogiginal Image",img)
cv2.moveWindow("Original Image",100,0)
print(img.shape)

# Split by color
height, width, channels = img.shape
blue, green, red = cv2.split(img)

### Split by color ###
cv2.imshow("Blue Channel", blue)
cv2.moveWindow("Blue Channed",0,height)

cv2.imshow("Red Channel", red)
cv2.moveWindow("Red Channed",0,height)

cv2.imshow("Green Channel", green)
cv2.moveWindow("Green Channed",0,height)

### Split by HSV ###
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
h,s,v = cv2.split(hsv)
hsv_image = np.concatenate((h,s,v), axis=1)
cv2.imshow("Hue, Saturation, Value Image", hsv_image)

### Convert to grayscale ###
gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray Image", gray_image)

# Wait untill enter any key in image frame
cv2.waitKey(0)
cv2.destroyAllWindows()
