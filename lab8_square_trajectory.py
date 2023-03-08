import socket
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

ROBOT_IP = '192.168.0.202'
CLIENT_IP = "192.168.0.41"
OPTITRACK_IP = "192.168.0.4"
ROBOT_ID = 202
STOP_COMMAND = 'stop'
STOP_MOTORS = 'CMD_MOTOR#0#0#0#0\n'

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

    angular_v = int(1200 * orientation_error)
    linear_v = int(800 * dist)

    u = np.array([linear_v - angular_v, linear_v + angular_v])
    u[u > 1500] = 1500
    u[u < -1500] = -1500

    # prevent robot from stalling before target
    if dist < .5:
        u[0 <= u.any() < 500] = 500
        u[-500 < u.any() <= 0] = -500

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

        while is_running:
            if ROBOT_ID in positions:
                robot_x, robot_y = get_position(ROBOT_ID)
                target_1 = (robot_x + 2, robot_y)
                target_2 = (robot_x + 2, robot_y + 2)
                target_3 = (robot_x, robot_y + 2)
                target_4 = (robot_x, robot_y)
                break

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(target_1, start_time)
                if command == STOP_COMMAND:
                    break

                s.send(command.encode('utf-8'))
                time.sleep(.1)

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(target_2, start_time)
                if command == STOP_COMMAND:
                    break

                s.send(command.encode('utf-8'))
                time.sleep(.1)

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(target_3, start_time)
                if command == STOP_COMMAND:
                    break

                s.send(command.encode('utf-8'))
                time.sleep(.1)

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(target_4, start_time)
                if command == STOP_COMMAND:
                    break

                s.send(command.encode('utf-8'))
                time.sleep(.1)

    except KeyboardInterrupt:
        pass

    s.send(STOP_MOTORS.encode('utf-8'))
    s.shutdown(2)
    s.close()
    streaming_client.shutdown()


if __name__ == '__main__':
    main()