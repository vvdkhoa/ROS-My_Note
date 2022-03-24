#!/usr/bin/env python 

from mimetypes import read_mime_types
import cv2
from cv2 import threshold


def read_image(image_name, as_gray):
    if as_gray:
        image = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)
    else:
        image = cv2.imread(image_name, cv2.IMREAD_COLOR)
    cv2.imshow('Image', image)
    return image


def basic_thresholding(gray_image, threshol_value):
    # THRESH_BINARY_INV: Hinh trang, nen den
    # THRESH_BINARY: Hinh den, nen trang
    ret, thresh_basic = cv2.threshold(gray_image, threshol_value, 255, 
                                        cv2.THRESH_BINARY_INV)
    cv2.imshow('Basic Binary Image',thresh_basic)


def adaptive_thresholding(gray_image, threshol_value):
    adaptive_threshold_image = cv2.adaptiveThreshold(gray_image, 
                                        255, 
                                        cv2.ADAPTIVE_THRESH_MEAN_C, 
                                        cv2.THRESH_BINARY_INV, 
                                        threshol_value, 
                                        2)
    cv2.imshow("Adaptive Threshold Image",adaptive_threshold_image)


def main():

    # 
    image_name = "src/open_cv/images/tomato.jpg"
    as_gray = True
    threshol_value = 115
    gray_image = read_image(image_name, as_gray)
    basic_thresholding(gray_image, threshol_value)
    adaptive_thresholding(gray_image, threshol_value)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()