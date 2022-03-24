#!/usr/bin/env python

import numpy as np
import cv2

video_capture = cv2.VideoCapture(0) # 0: the first video device
# video_capture = cv2.VideoCapture("my_video.mp4")

while(True):
    ret, frame = video_capture.read()

    cv2.imshow("Frame", frame)

    # Update every 10ms, exit when enter q in image frame
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
