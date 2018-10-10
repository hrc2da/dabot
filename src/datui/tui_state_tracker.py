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
    def __init__(self,node_name="tui_state"):
        rospy.init_node(node_name,anonymous=False)
        self.block_subscriber = rospy.Subscriber("/blocks", String, self.update_block_state)
        self.get_state_server = rospy.Service("get_tui_blocks", TuiState, self.get_block_state)
    def update_block_state(self,message):
        self.block_state = json.loads(message.data)
    def get_block_state(self,request):
        if request.frame_of_reference == "tuio":
            return TuiStateResponse(json.dumps(self.block_state))
        elif request.frame_of_reference == "arm":
            return TuiStateResponse(json.dumps(self.get_arm_frame_state()))
        else:
            raise ValueError("Must specify whether to retrieve state in tuio or arm frame.")
    def get_arm_frame_state(self):
        return []
    def shutdown_tracker(self):
        self.get_state_server.shutdown('shutting down tui state service')
        rospy.signal_shutdown("tearing down")
    
if __name__ == '__main__':
    t = TuiStateTracker()
    rospy.spin()