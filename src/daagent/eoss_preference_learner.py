#!/usr/bin/env python
"""
Eoss Preference Learner
======
Listen to /userconfigs and update a belief about user preference between science and cost
"""

import rospy
import numpy as np
import json

class EossPreferenceLearner:
    '''
    '''
    initial_probability = None
    transition_matrix = None
    previous_science = 0.0
    previous_cost = 0.0
    def __init__(self):
        rospy.init_node("eoss_preference_learner")
        self.config_subscriber = rospy.Subscriber("/userconfigs",String,self.handle_update)
        self.belief_publisher = rospy.Publisher("/eoss_preference_belief", String, queue_size=1) # for now let's publish the current belief as a string (e.g. SCIENCE or COST)
        self.initial_probability = 0.5
        self.transition_matrix = np.array([[.7, .3], [.3, .7]])
        self.delta_science = np.array([[.9 .1], [.1 .9]]) #[sci inc|1 inc|0], [dec|1 dec|0]
        self.delta_cost = np.array([[.1 .9], [.9 .1]]) #[inc|1 inc|0], [dec|1 dec|0]
        self.belief = self.initial_probability
    
    def timestep(self):
        self.belief = np.dot(self.transition_matrix, self.belief)

    def evidence(self, delta_science, delta_cost):
        if (delta_science >= 0):
            ds_row = 0
        else:
            ds_row = 1

        if (delta_cost >= 0):
            dc_cost = 0
        else:
            dc_cost = 1
        p_e  = (self.delta_science[ds_row] * self.delta_cost[dc_row] * self.belief.T).T    # Re-weigh belief probability by likelihood
        p_e = p_e / sum(p_e)                                   # Normalize to sum up to 1
        self.belief = p_e

    def filter(self, delta_science, delta_cost):
        self.timestep()
        self.evidence(delta_science, delta_cost)                                                        

    
    def handle_update(self, message):
        new_user_config = json.loads(message.data)
        config = new_user_config["config"]
        science = new_user_config["science"]
        cost = new_user_config["cost"]
        filter(science-previous_science, cost-previous_cost)
        previous_science = science
        previous_cost = cost                
        self.belief_publisher.publish(String(str(self.belief)))
                
        def reset(self, startprob=None):
        if startprob is not None:
            self._startprob = startprob
        if self._startprob is not None:
          self._belief = self._startprob
          
if __name__ == '__main__':
    p = EossPreferenceLearner()
    rospy.spin()