import time
from Motor import *
from Ultrasonic import *

# we want the robot to be 50 cm from the wall
TARGET_DIST = 50  

# position control proportion constant
K_VAL = 2

pwm = Motor()
ultrasonic = Ultrasonic()

def move_forward(speed):
    pwm.setMotorModel(speed, speed, speed, speed)

def stop():
    pwm.setMotorModel(0, 0, 0, 0)

if __name__ == '__main__':
    start_time = time.time()

    try:
        while True:
            dist = ultrasonic.get_distance()  # distance in cm

            if dist <= TARGET_DIST:
                stop()
                break

            error = dist - TARGET_DIST
            speed = int(K_VAL * error * 100)

            print(dist, speed)
            move_forward(speed)

    except KeyboardInterrupt:
        # end the program
        stop()

    print(f'Elapsed time: {time.time() - start_time} sec')

