from Motor import *
from Buzzer import *
from Led import *

pwm = Motor()
led = Led()
buzzer = Buzzer()

def move(wheel_1, wheel_2, wheel_3, wheel_4, duration):
    pwm.setMotorModel(wheel_1, wheel_2, wheel_3, wheel_4)
    time.sleep(duration)
    pwm.setMotorModel(0, 0, 0, 0)

def rotate_left():
    move(-1500, -1500, 1500, 1500, 1)

def beep(duration):
    buzzer.run('1')
    time.sleep(duration)
    buzzer.run('0')

move(1000, 1000, 1000, 1000, 2)  # move forward
rotate_left()
led.ledIndex(0x01, 255, 0, 0)  # red light

move(1000, 1000, 1000, 1000, 2)
rotate_left()
led.ledIndex(0x02, 0, 0, 255) # blue light

move(1000, 1000, 1000, 1000, 2)
rotate_left()
led.ledIndex(0x03, 0, 255, 0) # green light

move(1000, 1000, 1000, 1000, 2)
rotate_left()
led.ledIndex(0x04, 255, 255, 0) # yellow light

beep(1)

# turn off lights
led.colorWipe(led.strip, Color(0,0,0))


