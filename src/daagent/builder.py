#!/usr/bin/env python
"""
Builder
======
Gets the config from its search agent
Converts the config into a sequence of "moves"
Sends the moves to the arm one at a time
"""


class Builder:

    def __init__(self):
        pass

    def send_arm_goal(self, block, location):
        goal = MoveBlockGoal()
        pass

    def build(self, moves, cancellable=True, retry=False):
        for move in moves:
            try:
                send_arm_goal
            except Exception:
                if


for move in moves:

    #         try:
    #             self.handle_move_block(move)
    #         except Exception:
    #             if cancellable:
    #                 rospy.loginfo("Part of move failed, cancelling the rest.")
    #                 break
