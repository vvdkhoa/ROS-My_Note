#!/usr/bin/env python

import numpy as np
import cv2

# Read img
image_path = "src/open_cv/images/tree.jpg"
img = cv2.imread(image_path)

# Create windown
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)
cv2.imshow("Image",img)

# Wait untill enter any key in image frame
cv2.waitKey(0)

# Save to new file
cv2.imwrite("src/open_cv/images/tree-copy.jpg", img)
