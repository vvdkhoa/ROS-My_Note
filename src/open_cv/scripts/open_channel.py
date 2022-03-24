#!/usr/bin/env python

import numpy as np
import cv2

# Read img
image_path = "src/open_cv/images/tree.jpg"
img = cv2.imread(image_path)

# Img size
size = img.size
height, width, channels = img.shape
# length = img.shape[0]
# width = img.shape[1]
# channels = img.shape[2]
print("Image size: %s, height: %s, width: %s, channels: %s" % (size, height, width, channels))

# Create windown
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)
cv2.imshow("Image",img)

# Wait untill enter any key in image frame
cv2.waitKey(0)
