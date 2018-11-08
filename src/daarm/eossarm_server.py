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
from dabot.msg import MoveBlockGoal

class EossArmServer(DaArmServer):
    def __init__(self):
        # initialize the parent
        DaArmServer.__init__(self)

    def handle_EOSS_action(self,message):
        # message is {blockid, source={-1,0,1,2,3,4}, target = {-1,0,1,2,3,4}}
        source_point, source_zone = self.get_zone(message.source)
        target_point, target_zone = self.get_zone(message.target)
        goal = MoveBlockGoal()
        goal.id = message.id
        goal.source = source_point
        goal.source_x_tolerance = source_zone['x']
        goal.source_y_tolerance = source_zone['y']
        goal.target = target_point
        goal.target_x_tolerance = target_zone['x']
        goal.target_y_tolerance = target_zone['y']
        self.handle_move_block(goal)

    def get_zone(self,zone_id):
        
        
if __name__ == '__main__':
    e = EossArmServer()
