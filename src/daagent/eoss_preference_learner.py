#!/usr/bin/env python
"""
Eoss Preference Learner
======
Listen to /userconfigs and update a belief about user preference between science and cost
"""

import rospy
import numpy as np
import json
from std_msgs.msg import String
from scipy.stats import norm

class EossPreferenceLearner:
    '''
    '''
    initial_probability = None
    transition_matrix = None
    previous_science = 0.0
    previous_cost = 0.0
    counter = 0
    def __init__(self):
        rospy.init_node("eoss_preference_learner")
        self.config_subscriber = rospy.Subscriber("/userconfigs",String,self.handle_update)
        self.belief_publisher = rospy.Publisher("/eoss_preference_belief", String, queue_size=1) # for now let's publish the current belief as a string (e.g. SCIENCE or COST)
        self.initial_probability = np.array([0.5,0.5])
        self.transition_matrix = np.array([[0.7, 0.3], [0.3, 0.7]])
        self.delta_science = np.array([[.6, .2], [.4, .8]]) #[sci inc|1 inc|0], [dec|1 dec|0]
        self.delta_cost = np.array([[.9, .3], [.1, .7]]) #[inc|1 inc|0], [dec|1 dec|0]
        self.belief = self.initial_probability
        self.init_current_value_posteriors()
    
    def init_current_value_posteriors(self):
        self.p_science_given_max_science = norm(0.14,0.05)
        self.p_science_given_min_cost = norm(0.07,0.04)
        self.p_cost_given_max_science = norm(3000,1500)
        self.p_cost_given_min_cost = norm(1000,700)
        self.p_d_science_given_max_science = norm(0.04,0.02)
        self.p_d_science_given_min_cost = norm(0,0.04)
        self.p_d_cost_given_max_science = norm(400,300)
        self.p_d_cost_given_min_cost = norm(0,300)
    
    def get_science_posterior(self,science):
        return np.array([self.p_science_given_max_science.pdf(science),self.p_science_given_min_cost.pdf(science)])
    
    def get_cost_posterior(self,cost):
        return np.array([self.p_cost_given_max_science.pdf(cost),self.p_cost_given_min_cost.pdf(cost)])

    def get_d_science_posterior(self,d_science):
        return np.array([self.p_d_science_given_max_science.pdf(d_science),self.p_d_science_given_min_cost.pdf(d_science)])

    def get_d_cost_posterior(self,d_cost):
        return np.array([self.p_d_cost_given_max_science.pdf(d_cost),self.p_d_cost_given_min_cost.pdf(d_cost)])
    def timestep(self):
        self.belief = np.dot(self.transition_matrix, self.belief)
        rospy.loginfo("Belief at timestep: "+str(self.belief))
        

    def evidence(self, science, cost, delta_science, delta_cost):
        # if (delta_science >= 0):
        #     ds_row = 0
        # else:
        #     ds_row = 1

        # if (delta_cost >= 0):
        #     dc_row = 0
        # else:
        #     dc_row = 1
        # rospy.loginfo("DS_ROW: "+str(ds_row)+" DC_ROW: "+str(dc_row))
        #p_e  = (self.delta_science[ds_row] * self.delta_cost[dc_row] * self.get_science_posterior(science) * self.get_cost_posterior(cost) * self.belief.T).T    # Re-weigh belief probability by likelihood
        p_e  = (self.get_d_science_posterior(delta_science) * self.get_d_cost_posterior(delta_cost) * self.get_science_posterior(science) * self.get_cost_posterior(cost) * self.belief.T).T    # Re-weigh belief probability by likelihood
        rospy.loginfo("p_e: "+str(p_e))
        p_e = p_e / sum(p_e)                                   # Normalize to sum up to 1
        self.belief = p_e
        rospy.loginfo("Belief at evidence: "+str(self.belief))

    def filter(self, science, cost, delta_science, delta_cost):
        self.timestep()
        self.evidence(science, cost, delta_science, delta_cost)                                                        

    
    def handle_update(self, message):
        new_user_config = json.loads(message.data)
        config = new_user_config["config"]
        science = new_user_config["science"]
        cost = new_user_config["cost"]
        if self.counter > 0:
            self.filter(science, cost, science-self.previous_science, cost-self.previous_cost)
            self.belief_publisher.publish(String(json.dumps({"science":self.belief[0], "cost":self.belief[1]})))
        else:
            self.counter += 1
        self.previous_science = science
        self.previous_cost = cost                
        
                
    def reset(self, startprob=None):
        if startprob is not None:
            self._startprob = startprob
        if self._startprob is not None:
          self._belief = self._startprob
          
if __name__ == '__main__':
    p = EossPreferenceLearner()
    rospy.spin()