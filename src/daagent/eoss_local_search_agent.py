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
from dabot.srv import TargetDesigns, TargetDesignsResponse


class EossLocalSearchAgent:
    configurations = []
    evaluations = []
    criteria = [lambda(x): x[0] < 5000 and x[1] < -0.15]

    def __init__(self):
        rospy.init_node("local_search_agent", anonymous=False)
        self.evaluated_config_listener = rospy.Subscriber("/configurations", String, self.update_configurations)
        self.ready_publisher = rospy.Publisher("/agent_ready", String, queue_size=1)
        self.criteria_set_service = rospy.Service("/get_agent_designs", TargetDesigns, self.handle_get_criteria_set)

    def update_configurations(self, message):
        new_config = json.loads(message.data)
        self.configurations.append(new_config["config"])
        self.evaluations.append(new_config["cost"], -new_config["science"])
        self.pareto_truncate()

    def pareto_truncate(self):
        # truncate the pop
        pareto_indices = self.is_pareto_efficient_indexed(np.array(self.evaluations), False)
        self.configurations = [self.configurations[i] for i in pareto_indices]
        self.evaluations = [self.evaluations[i] for i in pareto_indices]
        criteria_set = self.get_criteria_set()
        if(len(criteria_set) > 0):
            self.ready_publisher.publish(String(json.dumps(criteria_set)))

    def handle_get_criteria_set(self, message):
        criteria_set = self.get_criteria_set
        if len(criteria_set) > 0:
            return TargetDesignsResponse(json.dumps(criteria_set))

    def get_criteria_set(self):
        criteria_set = [config for i, config in enumerate(
            self.configurations) if self.meets_criteria(self.evaluations[i])]
        return criteria_set

    def meets_criteria(self, evaluation):
        # return true if the config meets all criteria
        for criterion in self.criteria:
            if criterion(evaluation) is False:
                return False
        return True

    # stolen from https://stackoverflow.com/questions/32791911/fast-calculation-of-pareto-front-in-python
    def is_pareto_efficient_indexed(self, costs, return_mask=True):  # <- Fastest for many points
        """
        :param costs: An (n_points, n_costs) array
        :param return_mask: True to return a mask, False to return integer indices of efficient points.
        :return: An array of indices of pareto-efficient points.
                If return_mask is True, this will be an (n_points, ) boolean array
                Otherwise it will be a (n_efficient_points, ) integer array of indices.
        """
        is_efficient = np.arange(costs.shape[0])
        n_points = costs.shape[0]
        next_point_index = 0  # Next index in the is_efficient array to search for

        while next_point_index < len(costs):
            nondominated_point_mask = np.any(costs <= costs[next_point_index], axis=1)
            is_efficient = is_efficient[nondominated_point_mask]  # Remove dominated points
            costs = costs[nondominated_point_mask]
            next_point_index = np.sum(nondominated_point_mask[:next_point_index])+1

        if return_mask:
            is_efficient_mask = np.zeros(n_points, dtype=bool)
            is_efficient_mask[is_efficient] = True
            return is_efficient_mask
        else:
            return is_efficient
