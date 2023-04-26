import socket
import time
import math
import numpy as np
import networkx as nx
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

ROBOT_IP = '192.168.0.204'
CLIENT_IP = "192.168.0.47"
OPTITRACK_IP = "192.168.0.4"
ROBOT_ID = 204
STOP_COMMAND = 'stop'
STOP_MOTORS = 'CMD_MOTOR#0#0#0#0\n'

positions = {}
rotations = {}

POINTS = [
    (-2.8025026321411133, 2.034050464630127),
    (-1.6606419086456299, 1.9913979768753052),
    (-0.9018803834915161, 2.04339337348938),
    (-1.5610941648483276, 1.4344673156738281),
    (-2.3588311672210693, 1.4249827861785889),
    (-2.973353385925293, 1.4733598232269287),
    (-2.8268425464630127, 0.9526205658912659),
    (-2.1945114135742188, 0.8867706060409546),
    (-1.6091091632843018, 0.8781303763389587),
    (-1.589962363243103, 0.3326416015625),
    (-2.258037567138672, -0.04229611158370972),
    (-2.9624879360198975, -0.10876090824604034),
    (-1.556444525718689, -1.0077067613601685),
    (-2.343506097793579, -1.2544561624526978),
    (-3.1844584941864014, -1.4198737144470215),
]

EDGES = [
    (0, 4),
    (1, 4),
    (2, 4),
    (1, 3),
    (1, 5),
    (4, 8),
    (4, 6),
    (4, 9),
    (4, 11),
    (7, 9),
    (7, 11),
    (7, 10),
    (10, 12),
    (10, 14),
    (9, 13),
    (11, 13),
] 

G = nx.Graph()
G.add_nodes_from(POINTS)

for e in EDGES:
    u = POINTS[e[0]]
    v = POINTS[e[1]]
    dist = math.hypot(u[0] - v[0], u[1] - v[1])
    G.add_edge(u, v, weight=dist)


START = POINTS[1]
END = POINTS[12]

path = nx.shortest_path(G, source=START, target=END)

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

        for point in path:
            while is_running:
                if ROBOT_ID in positions and ROBOT_ID in rotations:
                    command = position_control(point, start_time)
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