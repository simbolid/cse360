import socket
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

ROBOT_IP = '192.168.0.205'
CLIENT_IP = "192.168.0.30"
OPTITRACK_IP = "192.168.0.4"
ROBOT_ID = 205
STOP_COMMAND = 'stop'
STOP_MOTORS = 'CMD_MOTOR#0#0#0#0\n'

TARGETS = [
    (-2.1865270137786865, 2.0245909690856934), # first turn
    (-2.8630740642547607, 2.1061336994171143), # second corner
    (-3.119551420211792, 1.6648197174072266),  # before duck
    (-3.158905506134033, 0.9007405638694763),  # after duck
    (-2.7547948360443115, 0.7081887722015381), # turn after duck
    (-1.982886552810669, 0.7081887722015381),  # last corner 
    (-1.882284164428711, 1.3756221532821655)   # original robot position
]

positions = {}
rotations = {}


def get_position(robot_id: int):
    three_dim = positions[robot_id]
    two_dim = three_dim[:2]
    return two_dim


def get_rotation(robot_id: int):
    return rotations[robot_id]


def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position

    # convert quaternion to euler angles (in radians)
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


def position_control(target, start_time):
    """instruct the robot to move to a given position"""
    robot_x, robot_y = get_position(ROBOT_ID)
    target_x, target_y = target
    x_diff = target_x - robot_x
    y_diff = target_y - robot_y

    dist = math.sqrt((x_diff ** 2) + (y_diff ** 2))

    # we got close enough to the target, so end the program
    if dist <= .175:
        return STOP_COMMAND

    robot_orientation_deg = get_rotation(ROBOT_ID)  # orientation in degrees
    robot_orientation = math.radians(robot_orientation_deg)  # convert to radians
    target_orientation = np.arctan2(y_diff, x_diff)
    orientation_diff = target_orientation - robot_orientation
    orientation_error = np.arctan2(np.sin(orientation_diff), np.cos(orientation_diff))

    angular_v = int(2700 * orientation_error)
    linear_v = int(2200 * dist)

    u = np.array([linear_v - angular_v, linear_v + angular_v])

    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])

    print(f"{time.time() - start_time:.3f}       {robot_x:.4f}/{target_x:.4f}        {robot_y:.3f}/{target_y:.3f}        ")

    return command


def main():
    # connect to robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_IP, 5000))
    print(f'Connected to robot {ROBOT_ID}')

    # connect to optitrack
    streaming_client = NatNetClient()
    streaming_client.set_client_address(CLIENT_IP)
    streaming_client.set_server_address(OPTITRACK_IP)
    streaming_client.set_use_multicast(True)
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    is_running = streaming_client.run()
    print('Connected to Optitrack\n')

    try:
        print('Time      X(Robot/Target)     Y(Robot/Target)')
        start_time = time.time()

        for t in TARGETS:
            while is_running:
                if ROBOT_ID in positions and ROBOT_ID in rotations:
                    command = position_control(t, start_time)
                    if command == STOP_COMMAND:
                        break

                    s.send(command.encode('utf-8'))
                    time.sleep(.1)

    except KeyboardInterrupt:
        pass

    print(f"Elapsed time: {time.time() - start_time}")

    s.send(STOP_MOTORS.encode('utf-8'))
    s.shutdown(2)
    s.close()
    streaming_client.shutdown()


if __name__ == '__main__':
    main()