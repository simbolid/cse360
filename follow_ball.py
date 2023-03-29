import numpy as np
import cv2
from picamera2 import Picamera2 
from Motor import *
import time

pwm = Motor()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

low_red = np.array([150, 130, 130])
high_red = np.array([180, 255, 255])
MIDDLE = 320  # horizontal middle of the resolution

try:
    while True:
        frame = picam2.capture_array()

        # filter out all colors except for red
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low_red, high_red)
        result = cv2.bitwise_and(frame, frame, mask = mask)

        # detect circles - circle detection more reliable than blob detection for the ball
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=100, param2=30,
                                   minRadius=30, maxRadius=0000)
        if circles is not None:
            # choose circle with largest radius
            circles = np.uint16(np.around(circles))
            circle = max(circles[0, :], key = lambda x: x[2])
            x, y, r = circle

            # outline circle
            cv2.circle(result, (x, y), r, (255, 0, 0), 3) 

            # control angular velocity depending on which side the ball is on
            angular_v = 200 * (MIDDLE - x)

            # control linear velocity depending on how far away ball is
            # (smaller circle radius --> ball farther away)
            linear_v = 400 * (200 - r)
            linear_v = max(linear_v, 300)  # prevent backwards movement

            u = np.array([linear_v - angular_v, linear_v + angular_v])
            pwm.setMotorModel(u[0], u[0], u[1], u[1])
            time.sleep(0.1)
        else:
            pwm.setMotorModel(0, 0, 0, 0)  

        # display feed
        cv2.imshow("feed", result)
        cv2.waitKey(1)
        
except KeyboardInterrupt:
    pwm.setMotorModel(0, 0, 0, 0)

