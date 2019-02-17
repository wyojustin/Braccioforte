'''
move N1 around in a pre determined pattern... Must be run from ArUco Mission directory
'''
from __future__ import print_function
import numpy
from numpy import linalg, newaxis, pi
import sys; sys.path.append('..')
from Common import util
from Common import safe_move
from Common.constants import DEG
from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo_one_run_python_api_code') ### initialize node

niryo = NiryoOne()
niryo.calibrate_auto()

lims = {0:[-20 * DEG, 20 * DEG],
        1:[-109 * DEG, -70 * DEG]}

_xs = numpy.linspace(lims[0][0], lims[0][1], 2)
_ss = numpy.linspace(lims[1][0], lims[1][1], 10)
xs, ss = util.raster_2d(_xs, _ss)

theta = [xs[0], ss[0], -pi/2 - ss[0], 0, 0, 0]
safe_move.set_speed(niryo, 100)
safe_move.move_joints(niryo, theta)

safe_move.set_speed(niryo, 2)

if True:
    for x, s in zip(xs, ss):
        theta[0] = x
        theta[1] = s
        theta[2] = -pi/2 - s
        print (x / DEG, s / DEG)
        safe_move.move_joints(niryo, theta, tries=10)
        
safe_move.relax(niryo)
safe_move.set_speed(niryo, 100)
