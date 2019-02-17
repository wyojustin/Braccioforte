#!/usr/bin/env python
from niryo_one_python_api.niryo_one_api import NiryoOneException
from constants import DEG

### limits
starts = [-3.05433, -1.91986, -1.397485, -3.05433, -1.74533, -2.57436]
stops =  [ 3.05433, 0.640187,  1.570796,  3.05433,  1.91986,  2.57436]

limits = zip(starts, stops)

def limit_move(angles):
    angles = [x for x in angles] ### copy angles
    for i in range(6):
        if angles[i] < starts[i]:
            angles[i] = starts[i]
        if angles[i] > stops[i]:
            angles[i] = stops[i]
    return angles

def move_joints(niryo, angles, tries=3):
    angles = limit_move(angles)
    for k in range(tries - 1):
        try:
            niryo.move_joints(angles)
            break
        except NiryoOneException:
            print ('move_joints attempt failed')
            pass
    else: ### allow exeption on last try
        niryo.move_joints(angles)

def move_pose(niryo, x, y, z, roll, pitch, yaw, tries=3):
    for k in range(tries - 1):
        try:
            niryo.move_pose(x, y, z, roll, pitch, yaw)
        except NiryoOneException:
            print ('move_pose attempt failed')
            pass
    else: ### allow exeption on last try
        niryo.move_pose(x, y, z, roll, pitch, yaw)        

def relax(niryo):
    niryo.activate_learning_mode(True)

def set_speed(niryo, percent):
    niryo.instance.set_arm_max_velocity(percent)
    
