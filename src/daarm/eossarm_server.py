#!/usr/bin/env python
"""
EOSS Arm Server
===================
EOSS-Specific actions (e.g. moving blocks between orbits)
"""
from daarm.daarm_server import DaArmServer
from dabot.srv import TuiState, TuiStateResponse
from dautils import get_ros_param
import rospy
from std_msgs.msg import String


class EossArmServer(DaArmServer):
    def __init__(self):
        # initialize the parent
        DaArmServer.__init__(self)


if __name__ == '__main__':
    e = EossArmServer()
