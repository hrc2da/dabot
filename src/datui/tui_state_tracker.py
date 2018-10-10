#!/usr/bin/env python
"""
TUI State
============
Maintains the current state of the TUI at a block (not configuration) level
Subscribes to: /blocks
Exposes services to get the block-state in TUIO or arm space
"""
import rospy
from dabot.srv import TuiStateResponse
from std_msgs.msg import String
import json
#from daarm import calibrate
class TuiStateTracker:
    """
    """
    block_state = []
    def __init__(self):
        rospy.init_node("tui_state",anonymous=False)
        self.blockSubscriber = rospy.Subscriber("/blocks", String, self.update_block_state)
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