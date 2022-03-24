#!/usr/bin/env python 

import cv2

image = cv2.imread("src/open_cv/images/tennisball05.jpg")
cv2.imshow('Original', image)

# Convert the image into the HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
cv2.imshow('HSV image', hsv)

# Find the upper and lower
yellowLower = (30,150,100)
yellowUpper = (50,255,255)

# Define a mask using the lower and upper
mask = cv2.inRange(hsv, yellowLower, yellowUpper)

cv2.imshow("Mask image", mask)

cv2.waitKey(0)
cv2.destroyAllWindows()
