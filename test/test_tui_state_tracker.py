#!/usr/bin/env python

import unittest
import sys, os
import json
from std_msgs.msg import String
from dabot.srv import TuiStateRequest
from os.path import dirname
sys.path.insert(0,os.path.join(dirname(dirname(__file__)),'src'))

from datui.tui_state_tracker import TuiStateTracker

class TestTuiState(unittest.TestCase):

    def test_update_block_state(self):
        tui = TuiStateTracker()
        block_state = json.dumps({"x":0.0,"y":0.0, "id":0})
        tui.update_block_state(String(block_state))
        
        self.assertEqual(tui.block_state.get('x'),0.0)
        self.assertEqual(tui.block_state.get('y'),0.0)
        self.assertEqual(tui.block_state.get('id'),0)

        tui.shutdown_tracker()
        

    def test_get_block_state(self):
        #print("testing get_block_state")
        tui = TuiStateTracker()

        #test without testing setter
        block_state = {"x":0.0,"y":0.0, "id":0}
        tui.block_state = block_state
        #test in tuio frame
        test_block_state = tui.get_block_state(TuiStateRequest("tuio")).tui_state
        self.assertDictEqual(block_state,json.loads(test_block_state))
        
        #test with the setter
        block_state123 = {"x":1.0,"y":2.0, "id":3}
        tui.update_block_state(String(json.dumps(block_state123)))
        #test in tuio frame
        test_block_state = tui.get_block_state(TuiStateRequest("tuio")).tui_state
        self.assertEqual(block_state123,json.loads(test_block_state))
        tui.shutdown_tracker()

if __name__ == '__main__':
    unittest.main()