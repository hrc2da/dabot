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
from dabot.msg import MoveEossBlockAction, MoveEossBlockGoal
from dabot.srv import TargetDesign, SleepControl, SleepControlResponse, TuiState, ArmCommand
from dautils import calibrate_arm
from dautils.pareto import pareto_sort
from kinova_msgs.msg import *
from kinova_msgs.srv import *
import numpy as np
import csv
import time
import os
import rospkg 
rospack = rospkg.RosPack()

class EossBuilder:
    '''
    queries the evaluator/search agent for a design to try
    converts the design into a list of moves, adds ,and removes
    sorts the list according to first "raw marginal benefit" and secondly "max delta"
    sends the commands one by one to the arm server, handling failures and successes

    sleeping behavior:
    when "awake" the builder will, with a configurable delay, ask the search agent for a design,
    block while it tries to build it, succeed or fail, delay, then repeat.

    when "asleep", the builder will only respond to external commands.

    when it receives an external command, the builder will sleep

    what happens if the builder is building when it receives an external command?

    actually, maybe the builder should only accept external commands when asleep.

    Let's have an option to interrupt and option to queue. If the external command wants to interrupt,
    the builder will have to cancel action cleanly and then sleep

    a simple workaround is also to ignore requests while not sleeping. Then, we still have to handle
    if a sleep command comes while building.

    '''
    building = False
    awake = False
    single_sensor_file_path = "config/single_sensor_outcomes.csv"
    build_delay = 5.0

    def __init__(self):
        # self.agent_ready_subscriber = rospy.Subscriber("/agent_ready", String, self.handle_agent_ready)
        # self.pause_subscriber = rospy.Subscriber("/pause_building", String, self.handle_pause)
        rospy.init_node("eoss_builder", anonymous=False)
        self.single_sensor_file = os.path.join(rospack.get_path('dabot'),self.single_sensor_file_path)
        self.load_marginal_benefits()
        self.sleep_service = rospy.Service("/eoss_builder_sleep_control", SleepControl, self.handle_sleep_control)
        rospy.wait_for_service('get_agent_design')
        self.get_design_to_build = rospy.ServiceProxy('get_agent_design', TargetDesign)
        rospy.wait_for_service('get_config_state')
        self.get_current_config = rospy.ServiceProxy('get_config_state', TuiState)
        self.eoss_action_client = actionlib.SimpleActionClient('move_eoss_block', MoveEossBlockAction)
        rospy.loginfo('Waiting for MoveEossBlockActionServer...')
        self.eoss_action_client.wait_for_server()
        rospy.loginfo('Connected to MoveEossBlockActionServer')
        self.status_publisher = rospy.Publisher("eoss_builder_status", String, queue_size=1)
        self.return_failed_config_publisher = rospy.Publisher("eoss_remove_from_visited", String, queue_size=1)

        # Service for homing the arm
        # home_arm_service = '/j2s7s300_driver/in/home_arm'
        home_arm_service = '/home_arm'
        self.home_arm_client = rospy.ServiceProxy(home_arm_service, ArmCommand)
        rospy.loginfo('Waiting for kinova home arm service')
        rospy.wait_for_service(home_arm_service)
        rospy.loginfo('Kinova home arm service server connected')
        self.home_arm_client()

        # main loop
        while not rospy.is_shutdown():
            if self.awake is True:
                #print("awake")
                # get a design from the search agent
                selected_design = ""
                self.status_publisher.publish(String("Waiting for a design to build."))
                while selected_design == "" and self.awake is True:
                    rospy.loginfo("requesting a design from eossbuilder")
                    selected_design_response = self.get_design_to_build(1)
                    selected_design = selected_design_response.design
                if self.awake is False:
                    continue # go back to sleep
                #build it
                rospy.loginfo("received design "+selected_design+". Building...")
                self.status_publisher.publish(String("Received design "+str(selected_design)+" to build. Cost: "+str(selected_design_response.cost)+" Science: "+str(selected_design_response.science)))
                self.build(selected_design)
                rospy.sleep(0.5)
                self.home_arm_client()
            else:
                pass
                #print("sleeping")
            #sleep
            rospy.sleep(self.build_delay)

    def handle_sleep_control(self, message):
        if message.command == "sleep":
            self.sleep(message.wait)
        elif message.command == "wake":
            self.wakeup()
        else:
            return SleepControlResponse("invalid sleep command")
        if self.awake is True:
            return SleepControlResponse("awake")
        else:
            return SleepControlResponse("asleep")

    def sleep(self, wait=True):
        self.status_publisher.publish(String("Received sleep signal. Going to sleep."))
        # cancel the current build if there is one and go to sleep
        self.awake = False
        # if wait is false, cancel the current action
        if wait is False:
            # call a service to pause the arm and cancel current goals
            self.eoss_action_client.cancel_all_goals()
            # I think I'll probably also need to pause the arm

        

    def wakeup(self):
        self.status_publisher.publish(String("Received wakeup signal."))
        self.awake = True


    def build(self,design):
        # get the current bitstring
        current_bitstring = self.get_current_config("eoss_config").tui_state
        rospy.loginfo("Comparing to current: "+current_bitstring)
        # generate an action sequence
        moves = EossBuilder.bitstring_to_actions(design, current_bitstring)
        # sort the sequence
        sorted_moves = self.sort_raw_marginal_benefit(moves)
        self.status_publisher.publish(String("Attempting the following sequence of moves: "+str(sorted_moves)))
        # send each of the actions in sequence, cancelling if any fail
        for move in sorted_moves:
            rospy.loginfo("Executing move "+str(move))
            self.status_publisher.publish(String("Executing move "+str(move)))
            if self.awake is False:
                rospy.loginfo("Went to sleep while performing build; canceling remaining moves")
                self.status_publisher.publish(String("Went to sleep while performing build; canceling remaining moves"))
                #self.return_failed_config_publisher.publish(String(design))
                return
                break
            self.eoss_action_client.send_goal(MoveEossBlockGoal(*move))
            if self.eoss_action_client.wait_for_result(rospy.Duration(90.0)):
                result = self.eoss_action_client.get_result()
                print(result)
                if result.success == 'success':
                    continue
                else:
                    rospy.loginfo("Build failed on "+str(move))
                    self.status_publisher.publish(String("Build failed on "+str(move)+": "+result.success))
                    #self.return_failed_config_publisher.publish(String(design))
                    return
                    break
                    #raise Exception("Build failed")
            else:
                # on timeout, cancel rest of moves
                self.eoss_action_client.cancel_all_goals()
                rospy.loginfo("Build timed out on "+str(move))
                self.status_publisher.publish(String("Build timed out on "+str(move)))
                #self.return_failed_config_publisher.publish(String(design))
                return
                break
        self.status_publisher.publish(String("Build was successful!"))
                #raise Exception("Build failed")
        # home the arm
        # homed = self.home_arm_client()

    def load_marginal_benefits(self):
        self.outcomes = {} #this is a dict keyed on config for all single block designs
        with open(self.single_sensor_file, 'r') as infile:
            csvreader = csv.reader(infile,delimiter=',')
            for i, (science,cost) in enumerate(csvreader):
                bitstring = '0'*i + '1' + '0'*(60-i-1)
                self.outcomes[bitstring] = np.array(map(float,[science,cost]))



    @staticmethod
    def bitstring_to_actions(target_bitstring,current_bitstring):
        # these are all lists of [block_id, source zone, target zone]
        # where staging is -1
        adds = []
        removes = []
        moves = []
        #get all the add/removes
        action_indices = [i for i,b in enumerate(target_bitstring) if b != current_bitstring[i]]
        # first sort into adds and removes
        for action in action_indices:
            block_id = action%12
            orbit = action/12
            if target_bitstring[action] == '1':
                # target is one, so it's an add
                adds.append([block_id,-1,orbit])
            else:
                # otherwise, it's 0, so it's a remove
                removes.append([block_id,orbit,-1])
        merged_adds = []
        merged_removes = []
        # then check for add/remove pairs by id as moves
        for i,add in enumerate(adds):
            # check if the block id is in removes
            for j,remove in enumerate(removes):
                #if the block id is the same, merge it
                if add[0] == remove[0] and j not in merged_removes:
                    # [id, source, target]
                    moves.append([add[0],remove[1],add[2]])
                    merged_adds.append(i)
                    merged_removes.append(j)
                    break
        # get rid of the add/remove pairs we merged into single moves
        # go backwards so we don't mess up indices while deleting
        # rospy.loginfo("Add "+str(adds))
        # rospy.loginfo("Remove"+str(removes))
        # rospy.loginfo("Move"+str(moves))
        merged_adds.sort()
        merged_removes.sort()
        for idxa in merged_adds[::-1]:
            adds.pop(idxa)
        for idxr in merged_removes[::-1]:
            removes.pop(idxr)
        
       
        
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
        for tier, actions in enumerate(action_tiers):
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
        

if __name__=='__main__':
    e = EossBuilder()





