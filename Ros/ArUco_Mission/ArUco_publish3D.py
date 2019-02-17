#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from control_msgs.msg import JointTrajectoryControllerState
from numpy import sin, cos, dot, array, transpose, pi, linalg, newaxis, arange, zeros
import signal, sys
import serial
import time
sys.path.append('..');
import Common.util

pub_aruco_3d = rospy.Publisher('/jevois/ArUco/3D', Float32MultiArray, queue_size=1)

def publish(poss):
    if not rospy.is_shutdown():
        if len(poss) == 6:
            pub_aruco_3d.publish(data=poss)

stop = False

rospy.init_node('ArUco_3D')

def signal_handler(signal, frame):
    print('^C')
    global stop
    stop = True
    sys.exit()
signal.signal(signal.SIGINT, signal_handler)


N_FIT = 3
last_positions = zeros((N_FIT, 6))
last_times = zeros(N_FIT)
def get_joints():
    if __n_pose[0] == 0:
        out = [0] * 6
    elif __n_pose[0] == 1:
        out = last_positions[0]
    else:
        ### compute position using poly fit on N_FIT points
        pows = arange(__n_pose[0])
        t = array(last_times[:__n_pose[0]]) - last_times[0]
        A = t[:,newaxis] ** pows[newaxis]
        coeff = dot(linalg.pinv(A), last_positions[:__n_pose[0]])
        rostime = rospy.get_rostime()
        current_time = rostime.secs + rostime.nsecs/1e9
        now = current_time - last_times[0]
        t_pows = now ** pows
        out = dot(t_pows, coeff)
    return out

def get_camera_pos():
    h1 = .183 ## high of axis 2 from ground
    h2 = .210 ## length between center of shoulder and center of elbow
    h3 = .078 ## offset of camer from axis 3
    
    joints = get_joints()
    theta = joints[0]
    rho = joints[1]
    psi = joints[2]

    R1 = Common.util.rotation(2, theta)
    R2 = Common.util.rotation(1, rho)
    orient_elbow = dot(R1, R2)
    R3 = Common.util.rotation(1, psi)

    rot = ([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) # camera xyz to real
    orient = dot(R1, dot(R2, dot(R3, rot)))

    p1 = array([0, 0, h1])
    p2 = p1 + dot(orient_elbow, [0, 0, h2])
    p3 = dot(orient, [0, -h3, .021])
    pos = p2 + p3
    return pos, orient, joints

DEG = pi / 180

#start = [30*DEG, 0, -40 * DEG, 0, -10 * DEG, 0]
#start = [0 * DEG, -45 * DEG, -45 * DEG, 0, 0, 0]

start = [0]  * 6


def cal_required():
    from niryo_one_msgs.msg import HardwareStatus
    hw_status = rospy.wait_for_message('niryo_one/hardware_status',
                                       HardwareStatus, timeout=5)
    return hw_status.calibration_needed

fmt = '%+.04f' 

count = 0
pos = [v for v in start]
MAX_WRIST = 0

__last_pos_index = [0]
__n_pose = [0]
def state_callback(data):
    last_positions[__last_pos_index[0]] = data.actual.positions
    last_times[__last_pos_index[0]] = data.header.stamp.secs + data.header.stamp.nsecs / 1e9
    __last_pos_index[0] = (__last_pos_index[0] + 1) % N_FIT
    if __n_pose[0] < N_FIT:
        __n_pose[0] += 1
    
def jevois_callback(data):
    global count
    count += 1
    
    if data.data[0] in set([42, 18, 12, 27, 43, 5]):
        vals = data.data
        value = vals[0]
        offset_cam = array(vals[1:4]) / 1000.
        pos_cam, orient_cam, joints = get_camera_pos()
        pos_abs = pos_cam + dot(orient_cam, offset_cam)
        x, y, z = pos_abs
        vals = [x, y, z, pos_cam[0], pos_cam[1], pos_cam[2]]
        publish(vals)
        print('%02d' % value, fmt % round(x, 3), fmt % round(y, 3), fmt % round(z, 3), end=' ')
        print(fmt % pos_cam[0], fmt % pos_cam[1], fmt % pos_cam[2], end=' ')
        print(fmt % offset_cam[0], fmt % offset_cam[1], fmt % offset_cam[2], end=' ')
        print(' '.join(map(str, orient_cam.ravel())))
        
        
        # print(get_joints())
        return

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber("jevois/ArUco/N3", Int16MultiArray, jevois_callback, queue_size=1)
    rospy.Subscriber("/niryo_one_follow_joint_trajectory_controller/state",
                     JointTrajectoryControllerState, state_callback, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
