#!/usr/bin/env python
"""
EossBuilder
======
Decides when to get a new eoss config from an agent
Gets the config from its search agent
Converts the config into a sequence of "moves"
Sends the moves to the arm as a sequence
"""


import rospy
import actionlib
from std_msgs.msg import String
from dabot.msg import CalibrateAction, CalibrateGoal
from dautils import calibrate_arm
from Tkinter import Tk, Label, Button


class EossBuilder:
    building = False
    paused = False

    def __init__(self):
        self.agent_ready_subscriber = rospy.Subscriber("/agent_ready", String, self.handle_agent_ready)
        self.pause_subscriber = rospy.Subscriber("/pause_building", String, self.handle_pause)
