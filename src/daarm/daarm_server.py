#!/usr/bin/env python

"""
DaArmServer
============
This provides an interface to control the Jaco arm via an action library
Basic actions:
1) Move a block from one region to another
    n.b. the basic arm knows NOTHING about orbits/voting precincts, etc.
    all the logic of translating a config to a block layout should be done
    in either an extended version of the arm or in a separate node
2) Perform a gesture from a dict of gestures stored in ROS Param server
3) Calibrate the arm space to tuio space mapping
"""

import rospy


class DaArmServer:
    """The basic, design problem/tui agnostic arm server
    """
    gestures = {}

    def __init__(self):
        rospy.init_node("daarm_server", anonymous=True)

    def handle_move_block(self, message):
        """msg format: {id: int,
                        from: {x: float,y: float},
                        to: {x: float, y: float}
        """


if __name__ == '__main__':
    arm = DaArmServer()
