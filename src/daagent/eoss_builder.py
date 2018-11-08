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
from dautils.pareto import pareto_sort
import numpy as np
import csv

class EossBuilder:
    '''
    queries the evaluator/search agent for a design to try
    converts the design into a list of moves, adds ,and removes
    sorts the list according to first "raw marginal benefit" and secondly "max delta"
    sends the commands one by one to the arm server, handling failures and successes

    '''
    building = False
    paused = False
    single_sensor_file = "../config/single_sensor_outcomes.csv"

    def __init__(self):
        self.agent_ready_subscriber = rospy.Subscriber("/agent_ready", String, self.handle_agent_ready)
        self.pause_subscriber = rospy.Subscriber("/pause_building", String, self.handle_pause)
        rospy.waitForService('get_agent_designs')
        self.get_designs_to_build = rospy.ServiceProxy('get_agent_designs')
        rospy.waitForService('get_config_state')
        self.get_current_config = rospy.ServiceProxy('get_config_state', TuiState)

    def load_marginal_benefits(self):
        self.outcomes = {} #this is a dict keyed on config for all single block designs
        with open(self.single_sensor_file, 'r') as infile:
            csvreader = csv.reader(infile,delimiter=',')
            for i, (science,cost) in enumerate(csvreader):
                bitstring = '0'*i + '1' + '0'*(60-i-1)
                self.outcomes[bitstring] = np.array([science,cost])
    def bitstring_to_actions(self,bitstring):
        # these are all lists of [block_id, source zone, target zone]
        # where staging is -1
        adds = []
        removes = []
        moves = []
        current_bitstring = self.get_current_config().tui_state
        #get all the add/removes
        action_indices = [i for i,b in enumerate(target_btstr) if b != cur_btstr[i]]
        # first sort into adds and removes
        for action in action_indices:
            block_id = action%12
            orbit = action/12
            if bitstring[action] == '1':
                # target is one, so it's an add
                adds.append([block_id,-1,orbit])
            else:
                # otherwise, it's 0, so it's a remove
                removes.append([block_id,orbit,-1])
        merged = []
        # then check for add/remove pairs by id as moves
        for i,add in enumerate(adds):
            # check if the block id is in removes
            for j,rm in enumerate(removes):
                #if the block id is the same, merge it
                if add[0] == remove[0]:
                    # [id, source, target]
                    moves.append([add[0],remove[1],add[2]])
                    merged.append([i,j])
                    break
        # get rid of the add/remove pairs we merged into single moves
        for m in merged:
            adds.pop(m[0])
            removes.pop(m[1])
        # now put everything into a giant moves array
        moves.extend(adds)
        moves.extend(removes)
        return moves

    def sort_raw_marginal_benefit(self,actions):
        sorted_sequence = []
        # get a science/cost for each action
        outcomes = [self.get_raw_marginal_benefit(action) for action in actions]
        # pareto_sort
        action_tiers,outcome_tiers = pareto_sort(actions,outcomes) #if there's problems, make sure we're passing np arrays
        # within each tier, sort by max of science/cost difference (we are favoring extreme moves)
        for tier, actions in action_tiers:
            #sort actions based on outcomes in outcome_tiers[tier]
            #actually no sort for now,let's just append
            sorted_sequence.extend(actions)
        return sorted_sequence

    def get_block_bitstring(self,block_id,location):
        if location < 0:
            return '0'*60
        else:
            bit_index = 12*location + block_id
            return '0'*bit_index + '1' + '0'*(60-bit_index-1)

    def get_raw_marginal_benefit(self,action):
        block_id, source, target = action
        if source < 0:
            source_val = np.array([0,0])
        else:
            source_val = self.outcomes[self.get_block_bitstring(block_id,source)]
        if target < 0:
            target_val = np.array([0,0])
        else:
            target_val = self.outcomes[self.get_block_bitstring(block_id,target)]

        return target_val - source_val
        







