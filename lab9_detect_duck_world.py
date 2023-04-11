### this code works only on the raspberry
import math
import time
import numpy as np
import cv2
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

def get_duck_position_robot(s, u):
    """
    get the position of the duck relative to the front of the robot.

    @param s: the size of the duck blob 
    @param u: the distance of the blob away from the center of the viewport

    @returns polar coordinates (d, theta), theta in radians
    """
    d = 17.122 * math.log(1/s) + 182.47
    theta = .0892 * u + 2.109
    theta = np.deg2rad(theta)
    return (d, theta)

start_time = time.time()

# (d, theta, x, y) relative to robot
log = []
log.append(f"start time: {start_time}\n")
log.append("time    d    theta    x    y\n")

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

                # d in cm, theta in radians
                d, theta = get_duck_position_robot(area, u)

                # orientation:
                # robot --> 
                # +x -->
                # +y down
                duck_x = d * np.cos(theta)  # in cm
                duck_y = d * np.sin(theta)

                elapsed_time = time.time() - start_time
                log.append(f"{elapsed_time:.4f}    {d:.4f}    {theta:.4f}    {duck_x:.4f}    {duck_y:.4f}\n")

        # cv2.imshow('image', image)
        cv2.waitKey(1)

except KeyboardInterrupt:
    pass
    # cv2.destroyWindow('image')

with open("duck_data.txt", "w") as file:
    file.writelines(log)


