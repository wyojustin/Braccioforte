#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo_one_run_python_api_code') ### initialize node
niryo = NiryoOne()
niryo.calibrate_auto()
niryo.activate_learning_mode(True)

