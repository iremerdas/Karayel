import cv2
import numpy as np
from collections import deque


def color_detect(frame):
    # nesne merkezini depolayacak veri tipi
    buffer_size = 16  # deque nun boyutu
    pts = deque(maxlen=buffer_size)  # nesnenin merkez noktaları

    contours = []

    # renk aralıkları HSV formatında
    lower_red1 = np.array([1, 170, 100])
    upper_red1 = np.array([5, 255, 255])
    lower_red2 = np.array([170, 170, 100])
    upper_red2 = np.array([179, 255, 255])
    lower_yellow = np.array([25, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([70, 255, 255])

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    into_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask_red1 = cv2.inRange(into_hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(into_hsv, lower_red2, upper_red2)
    mask_red = mask_red1 + mask_red2
    mask_yellow = cv2.inRange(into_hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(into_hsv, lower_green, upper_green)

    mask_red = cv2.erode(mask_red, None, iterations=2)
    mask_red = cv2.dilate(mask_red, None, iterations=2)
    _, mask_red = cv2.threshold(mask_red, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
    mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
    mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
    mask_green = cv2.erode(mask_green, None, iterations=2)
    mask_green = cv2.dilate(mask_green, None, iterations=2)

    (contours_red, _) = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (contours_yellow, _) = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (contours_green, _) = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_contour_area = 200
    contours_red = [cnt for cnt in contours_red if cv2.contourArea(cnt) > min_contour_area]
    contours_yellow = [cnt for cnt in contours_yellow if cv2.contourArea(cnt) > min_contour_area]
    contours_green = [cnt for cnt in contours_green if cv2.contourArea(cnt) > min_contour_area]

    epsilon = 0.03
    contours_red = [cv2.approxPolyDP(cnt, epsilon * cv2.arcLength(cnt, True), True) for cnt in contours_red]
    contours_yellow = [cv2.approxPolyDP(cnt, epsilon * cv2.arcLength(cnt, True), True) for cnt in contours_yellow]
    contours_green = [cv2.approxPolyDP(cnt, epsilon * cv2.arcLength(cnt, True), True) for cnt in contours_green]

    contours.append((contours_red, contours_yellow, contours_green))

    return contours

