#!/usr/bin/env python
"""
TUI State Tracker
==================
Maintains the current state of the TUI at a block (not configuration) level
Subscribes to: /blocks
Exposes services to get the block-state in TUIO or arm space
"""
import rospy
from dabot.srv import TuiState, TuiStateResponse
from std_msgs.msg import String
import json
#from daarm import calibrate


class TuiStateTracker:
    """
    """
    block_state = []

    def __init__(self, node_name="tui_state"):
        rospy.init_node(node_name, anonymous=False)
        self.block_subscriber = rospy.Subscriber("/blocks", String, self.update_block_state)
        self.get_state_server = rospy.Service("get_tui_blocks", TuiState, self.get_block_state)
        self.table_bounds = self.get_table_bounds()
        self.arm_bounds = self.get_arm_bounds()

    def update_block_state(self, message):
        self.block_state = json.loads(message.data)

    def get_table_bounds(self):
        table_bounds = {}
        table_bounds['min_x'] = self.get_ros_param('tuio_min_x', 'Table bounds must be configured')
        table_bounds['max_x'] = self.get_ros_param('tuio_max_x', 'Table bounds must be configured')
        table_bounds['min_y'] = self.get_ros_param('tuio_min_y', 'Table bounds must be configured')
        table_bounds['max_y'] = self.get_ros_param('tuio_max_y', 'Table bounds must be configured')
        table_bounds['x_dist'] = table_bounds['max_x'] - table_bounds['min_x']
        table_bounds['y_dist'] = table_bounds['max_y'] - table_bounds['min_y']
        return table_bounds

    def get_arm_bounds(self):
        arm_bounds = {}
        arm_bounds['x_dist'] = self.get_ros_param('arm_x_dist', 'Arm reachable range must be configured')
        arm_bounds['y_dist'] = self.get_ros_param('arm_y_dist', 'Arm reachable range must be configured')

        if rospy.has_param('arm_x_min') and rospy.has_param('arm_y_min'):
            arm_bounds['x_min'] = rospy.get_param('arm_x_min')
            arm_bounds['y_min'] = rospy.get_param('arm_y_min')
        else:
            arm_bounds['x_min'], arm_bounds['y_min'] = calibrate()  # This needs to be implemented

        arm_bounds['x_max'] = arm_bounds['x_min'] + arm_bounds['x_dist']
        arm_bounds['y_max'] = arm_bounds['y_min'] + arm_bounds['y_dist']
        return arm_bounds

    def get_block_state(self, request):
        if request.frame_of_reference == "tuio":
            return TuiStateResponse(json.dumps(self.block_state))
        elif request.frame_of_reference == "arm":
            return TuiStateResponse(json.dumps(self.get_arm_frame_state()))
        else:
            raise ValueError("Must specify whether to retrieve state in tuio or arm frame.")

    def get_arm_frame_state(self):
        return [self.translate(b) for b in self.block_state]

    def get_ros_param(self, param, err_msg):
        if(rospy.has_param(param)):
            return rospy.get_param(param)
        else:
            raise ValueError(err_msg)

    def translate(self, block):
        # Configure the translation of a block from tuio space (0 to 1 in camera view) to arm space ( meters away from base )
        x_scalar, y_scalar = self.scaleT2J(block['x'], block['y'])
        arm_bounds = self.arm_bounds
        x_arm_space = x_scalar * arm_bounds['x_dist'] + arm_bounds['x_min']
        y_arm_space = y_scalar * arm_bounds['y_dist'] + arm_bounds['y_min']
        return {"x": x_arm_space, "y": y_arm_space, "id": block['id']}

    def scaleT2J(self, x, y):
        # scales block coords from table's observable range into 0 to 1 scalar for arm space
        table_bounds = self.table_bounds
        return ((table_bounds['max_x']-x)/table_bounds['x_dist'], (table_bounds['max_y']-y)/table_bounds['y_dist'])

    def shutdown_tracker(self):
        self.get_state_server.shutdown('shutting down tui state service')
        rospy.signal_shutdown("tearing down")


if __name__ == '__main__':
    t = TuiStateTracker()
    rospy.spin()
