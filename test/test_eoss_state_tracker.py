#!/usr/bin/env python

import unittest
import sys, os
import json
import rospy
from std_msgs.msg import String
from dabot.srv import TuiStateRequest
from os.path import dirname
sys.path.insert(0,os.path.join(dirname(dirname(__file__)),'src'))

from datui.eoss_state_tracker import EossStateTracker

class TestEossState(unittest.TestCase):


    def test_set_workspace_bounds(self):
        bounds = {
            "ORBITS_MIN": -0.6,
            "ORBITS_MAX": -0.2,
            "ORBITS_LEFT": 0.25,
            "ORBITS_RIGHT": 0.45
        }
        rospy.set_param('WORKSPACE_BOUNDS',bounds)
        eoss = EossStateTracker()
        eoss.set_workspace_bounds(bounds)
        orbit_height = (bounds.get("ORBITS_MAX")-bounds.get("ORBITS_MIN"))/5
        self.assertAlmostEqual(orbit_height,0.08) #sanity check
        self.assertAlmostEqual(orbit_height,eoss.orbit_height)
        self.assertEqual(bounds.get("ORBITS_LEFT"),eoss.orbits_left)
        self.assertEqual(bounds.get("ORBITS_RIGHT"),eoss.orbits_right)
        
        self.assertEqual(5,len(eoss.orbits))
        self.assertEqual(bounds.get("ORBITS_MAX"),eoss.orbits[0][1])
        self.assertAlmostEqual(bounds.get("ORBITS_MIN"),eoss.orbits[-1][0])
        self.assertAlmostEqual(bounds.get("ORBITS_MAX")-orbit_height,eoss.orbits[0][0])


        eoss.shutdown_tracker()
        self.teardown()

    


    def teardown(self):
        if rospy.has_param('WORKSPACE_BOUNDS'):
            rospy.delete_param('WORKSPACE_BOUNDS')
        

if __name__ == '__main__':
    unittest.main()