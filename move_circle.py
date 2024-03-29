import socket
import time
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

ROBOT_IP = '192.168.0.203'
CLIENT_IP = "192.168.0.29"
OPTITRACK_IP = "192.168.0.4"
ROBOT_ID = 205
STOP_COMMAND = 'stop'
STOP_MOTORS = 'CMD_MOTOR#0#0#0#0\n'

positions = {}
rotations = {}
log = []

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


def position_control(start_time):
    """instruct the robot to move in a circle"""
    robot_x, robot_y = get_position(ROBOT_ID)

    angular_v = 700
    linear_v = 800

    u = np.array([linear_v - angular_v, linear_v + angular_v])
    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])

    elapsed_time = time.time() - start_time

    if elapsed_time > 60:
        return STOP_COMMAND

    log.append(f"{elapsed_time:.3f}     {robot_x:.4f}    {robot_y:.4f}\n")

    return command


def write_log():
    with open("robot_position_data.txt", "w") as file:
        file.writelines(log)


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
        start_time = time.time()
        log.append(f"start time = {start_time:.3f}\n")
        log.append(" Time      RobotX    RobotY\n")

        while is_running:
            if ROBOT_ID in positions and ROBOT_ID in rotations:
                command = position_control(start_time)
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

    write_log()


if __name__ == '__main__':
    main()
