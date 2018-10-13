#!/usr/bin/env python

"""
TuioArmCalibrator
============
This provides a calibration action that will take a block,
place it on the table, update calibration rosparams,
and return and calibrate the params
"""

import rospy
import actionlib
from dabot.msg import CalibrateAction, CalibrateFeedback, CalibrateResult
from dabot.srv import TuiState
from moveit_commander import RobotCommander, MoveGroupCommander


class TuioArmCalibrator:
    def __init__(self):

    def calibrate(self, message):
