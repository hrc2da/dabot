#!/usr/bin/env python
"""
EossLocalSearchAgent
======
Subscribe to /configurations.
Supplies a "best" config based on all the ones seen so far
Publishes on /ready when it meets its own design quality criteria
"""


import rospy
import json
import numpy as np
import actionlib
from std_msgs.msg import String
from dabot.msg import CalibrateAction, CalibrateGoal
from dautils import calibrate_arm
from dabot.srv import TargetDesign, TargetDesignResponse, TuiState, TuiStateResponse, FindEossBlock
from dautils.pareto import pareto_truncate
from daagent.eoss_builder import EossBuilder 
bitstring_to_actions = EossBuilder.bitstring_to_actions
from random import randint

class EossLocalSearchAgent:
    configurations = []
    evaluations = []
    attempted = [] # keep a visited list to avoid repeating moves
    pareto_front = []
    pareto_evaluations = []
    eval_dict = {}
    criteria = [lambda(x): x[0] < 5000 and x[1] < -0.15]

    def __init__(self):
        rospy.init_node("eoss_local_search_agent", anonymous=False)
        self.evaluated_config_listener = rospy.Subscriber("/configs", String, self.update_configurations)
        # self.ready_publisher = rospy.Publisher("/agent_ready", String, queue_size=1)
        # Service to get TUI State
        rospy.wait_for_service('get_config_state')
        self.get_current_config = rospy.ServiceProxy('get_config_state', TuiState)
        rospy.wait_for_service('check_move')
        self.check_move = rospy.ServiceProxy('check_move', FindEossBlock)
        #service to get the set of designs that match the criteria, the criteria set
        #self.criteria_set_service = rospy.Service("/get_agent_designs", TargetDesigns, self.handle_get_criteria_set)
        #service to get the best current design
        self.get_target_design_service = rospy.Service("/get_agent_design", TargetDesign, self.handle_get_target_design)

    def update_configurations(self, message):
        new_config = json.loads(message.data)
        if(self.eval_dict.get(new_config["config"],None) != None):
            return
        else:
            self.eval_dict[new_config["config"]] = (new_config["cost"],-new_config["science"])
        self.configurations.append(new_config["config"])
        self.evaluations.append([new_config["cost"], -new_config["science"]])
        dd,do,nd,no = pareto_truncate(self.configurations, self.evaluations)
        self.pareto_front = dd
        self.pareto_evaluations = do
        #print(self.pareto_evaluations)

    # def pareto_truncate(self):
    #     # truncate the pop
    #     pareto_indices = self.is_pareto_efficient_indexed(np.array(self.evaluations), False)
    #     self.configurations = [self.configurations[i] for i in pareto_indices]
    #     self.evaluations = [self.evaluations[i] for i in pareto_indices]
    #     criteria_set = self.get_criteria_set()
    #     if(len(criteria_set) > 0):
    #         self.ready_publisher.publish(String(json.dumps(criteria_set)))

    def handle_get_target_design(self, message):
        # remove the ones already tried
        # note that by doing this here we are limiting our options
        # if we remove when pareto sorting, we get more designs but include less good ones
        # for now, let's go for better quality and less quantity
        pareto_front = self.pareto_front[:] #this may be paranoid and expensive, but freeze pf
        print("PARETO_FRONT:", pareto_front)
        # there is a lot going on here. (e.g. we're using bitstring for state, while check_move uses blocks...)
        current_config = self.get_current_config("eoss_config").tui_state
        feasible_designs = [design for design in pareto_front if len(self.get_undoable_moves(design)) == 0]
        print("FEASIBLE_DESIGNS:", feasible_designs)
        if len(feasible_designs) == 0:
            return TargetDesignResponse("",0.0,0.0)
        # sort the pareto set by diff
        sorted_front = self.sort_by_diff(feasible_designs,exclusions=self.attempted)
        print("SORTED FRONT:",sorted_front.keys())
        sorted_front.pop("0",-1) # first remove the current config if it's in there
        if len(sorted_front) == 0:
            return TargetDesignResponse("",0.0,0.0)
        # choose one and return it, not sure if I have to cast to int here, but just being safe
        shortest_move = min(map(int,sorted_front))
        choices = sorted_front[str(shortest_move)]
        choice = randint(0,len(choices)-1)
        self.attempted.append(choices[choice])
        choice_outcomes = self.eval_dict.get(choices[choice])
        return TargetDesignResponse(choices[choice],choice_outcomes[0],-choice_outcomes[1])

 

    def get_undoable_moves(self,target_bitstring):
        current_config = self.get_current_config("eoss_config").tui_state
        actions = bitstring_to_actions(target_bitstring,current_config)
        # return the number of undoable_actions
        return [action for action in actions if self.check_move(action[0],action[1]).num_found <= 0 ]

    # def handle_get_criteria_set(self, message):
    #     criteria_set = self.get_criteria_set
    #     if len(criteria_set) > 0:
    #         return TargetDesignsResponse(json.dumps(criteria_set))

    # def get_criteria_set(self):
    #    criteria_set = [config for i, config in enumerate(
    #         self.configurations) if self.meets_criteria(self.evaluations[i])]
    #     return criteria_set

    # def meets_criteria(self, evaluation):
    #     # return true if the config meets all criteria
    #     for criterion in self.criteria:
    #         if criterion(evaluation) is False:
    #             return False
    #     return True

    def sort_by_diff(self, to_sort, exclusions = []):
        """
        This function sorts by the number of moves (add, remove, move) it would take to realize a config
        It returns a dict of lists of designs keyed by the number of moves
        """
        designs_to_sort = to_sort[:] # ignore updates while sorting
        if len(exclusions) > 0:
            designs_to_sort = [design for design in designs_to_sort if design not in exclusions]
        current_config = self.get_current_config("eoss_config").tui_state
        move_lengths = [len(bitstring_to_actions(config,current_config)) for config in designs_to_sort]
        #return [config for _, config in sorted(zip(move_lengths,self.pareto_front), key=lambda pair: pair[0])]
        designs_by_moves = {}
        for i,length in enumerate(move_lengths):
            key = str(length)
            if key in designs_by_moves:
                designs_by_moves[key].append(designs_to_sort[i])
            else:
                designs_by_moves[key] = [designs_to_sort[i]]
        return designs_by_moves

    

    # stolen from https://stackoverflow.com/questions/32791911/fast-calculation-of-pareto-front-in-python
    # def is_pareto_efficient_indexed(self, costs, return_mask=True):  # <- Fastest for many points
    #     """
    #     :param costs: An (n_points, n_costs) array
    #     :param return_mask: True to return a mask, False to return integer indices of efficient points.
    #     :return: An array of indices of pareto-efficient points.
    #             If return_mask is True, this will be an (n_points, ) boolean array
    #             Otherwise it will be a (n_efficient_points, ) integer array of indices.
    #     """
    #     is_efficient = np.arange(costs.shape[0])
    #     n_points = costs.shape[0]
    #     next_point_index = 0  # Next index in the is_efficient array to search for

    #     while next_point_index < len(costs):
    #         nondominated_point_mask = np.any(costs <= costs[next_point_index], axis=1)
    #         is_efficient = is_efficient[nondominated_point_mask]  # Remove dominated points
    #         costs = costs[nondominated_point_mask]
    #         next_point_index = np.sum(nondominated_point_mask[:next_point_index])+1

    #     if return_mask:
    #         is_efficient_mask = np.zeros(n_points, dtype=bool)
    #         is_efficient_mask[is_efficient] = True
    #         return is_efficient_mask
    #     else:
    #         return is_efficient

if __name__ == '__main__':
    e = EossLocalSearchAgent()
    rospy.spin()
