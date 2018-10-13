"""
FakeTui
========
Simply emulates a table for testing
It listens to /arm_action_beliefs and
maintains a list of current blocks and
publishes to /blocks when it updates
"""

import rospy
from std_msgs.msg import String
from dautils import get_tuio_bounds, get_arm_bounds, arm_to_tuio
import json
import numpy as np


class FakeTui:
    """
    """
    blocks = []
    arm_calibrated = False
    detection_threshold = 0.1

    def __init__(self):
        rospy.init_node("fake_tui", anonymous=False)
        self.block_publisher = rospy.Publisher("/blocks", String)
        self.arm_subscriber = rospy.Subscriber("/arm_action_beliefs", String, self.update_blocks)

    def update_blocks(self, message):
        rospy.loginfo("UPDATING BLOCKS")
        action_belief = json.loads(message.data)
        new_block = action_belief.block
        action = action_belief.action
        if self.arm_calibrated is False:
            tuio_block = {'x': 0.5, 'y': 0.5, 'id': new_block['id']}
            self.arm_calibrated = True
        else:
            tuio_x, tuio_y = arm_to_tuio(new_block['x'], new_block['y'], get_tuio_bounds(), get_arm_bounds())
            tuio_block = {'x': tuio_x, 'y': tuio_y, 'id': new_block['id']}
        if(action == 'add'):
            self.blocks.append(tuio_block)
        else:
            self.remove_block(tuio_block)
        rospy.loginfo(self.blocks)
        self.block_publisher.publish(String(json.dumps(self.blocks)))

    def remove_block(self, block):
        proximities = [np.linalg.norm(np.array([b['x'], b['y']])-np.array([block['x'], block['y']]))
                       for b in self.blocks]
        closest_block = np.argmin(proximities)
        if np.min(proximities) < self.detection_threshold:
            self.blocks.pop(closest_block[0])


if __name__ == '__main__':
    f = FakeTui()
    rospy.spin()
