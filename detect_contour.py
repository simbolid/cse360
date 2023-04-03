### this code works only on the raspberry
import numpy as np
import cv2
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

try:
    while True:
        frame = picam2.capture_array()

        # filter out all colors except for yellow
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_color = np.array([0, 150, 150])
        high_color = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, low_color, high_color)
        image = cv2.bitwise_and(frame, frame, mask = mask)
        image[image > 0] = 220  # combine colors

        # find contours
        blur = cv2.medianBlur(image, 5)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # (size, u)
        duck_data = []

        min_area = 500
        for c in cnts:
            area = cv2.contourArea(c)
            if area > min_area:
                cv2.drawContours(image, [c], -1, (36, 255, 12), 2)
                
                # compute contour distance from center
                frame_center = image.shape[:2][1] / 2  # width divided by 2
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                u = cX - frame_center

                duck_data.append((area, u))

        print(duck_data)

        cv2.imshow('image', image)
        cv2.waitKey(1)
except KeyboardInterrupt:
    cv2.destroyWindow('image')

