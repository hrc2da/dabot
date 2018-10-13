#!/usr/bin/env python

import unittest
import sys
import os
import json
import rospy
from std_msgs.msg import String

from os.path import dirname
sys.path.insert(0, os.path.join(dirname(dirname(__file__)), 'src'))
from dabot.msg import CalibrateAction, CalibrateGoal
from daarm.daarm_server import DaArmServer
import actionlib


class TestCalibration(unittest.TestCase):

    def test_calibration(self):
        arm = DaArmServer()
        client = actionlib.SimpleActionClient('arm_calibration', CalibrateAction)
        client.wait_for_server()
        goal = CalibrateGoal("Let's do this!")
        client.send_goal(goal)
        client.wait_for_result()
        print("result", client.get_result())


if __name__ == '__main__':
    unittest.main()
