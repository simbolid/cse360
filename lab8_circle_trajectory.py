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
# CENTER_X = 4.3148
# CENTER_Y = -0.2826

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

def position_control(start_time, center_x, center_y):
    t = time.time() - start_time

    if t >= 15:
       return STOP_COMMAND

    robot_x, robot_y = get_position(ROBOT_ID)

    target_x = np.cos(t) + center_x
    target_y = np.sin(t) + center_y

    trajectory_ux = -np.sin(t)
    trajectory_uy = np.cos(t)

    ux = trajectory_ux + 800 * (target_x - robot_x)
    uy = trajectory_uy + 800 * (target_y - robot_y)
    
    u = np.array([ux, uy])

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
                center_x, center_y = robot_x + 1, robot_y
                break

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(start_time, center_x, center_y)
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