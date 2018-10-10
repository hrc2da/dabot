#!/usr/bin/env python
"""
EOSS State Tracker
===================
Maintains the current state of the TUI
including in terms of EOSS configuration
"""
from datui.tui_state_tracker import TuiStateTracker
from dabot.srv import TuiState, TuiStateResponse
import rospy


class EossStateTracker(TuiStateTracker):
    """
    """
    NUM_ORBITS = 5
    orbits = []

    def __init__(self):
        if(rospy.has_param("WORKSPACE_BOUNDS")):
            self.set_workspace_bounds(rospy.get_param("WORKSPACE_BOUNDS"))
        else:
            raise ValueError("Orbit Bounds are not defined. Shutting down...")
        # initialize the parent
        TuiStateTracker.__init__(self, node_name="tui_state")
        self.get_config_server = rospy.Service("get_config_state", TuiState, self.get_config_state)

    def set_workspace_bounds(self, bounds):
        self.orbits_min = bounds["ORBITS_MIN"]
        self.orbits_max = bounds["ORBITS_MAX"]
        self.orbits_left = bounds["ORBITS_LEFT"]
        self.orbits_right = bounds["ORBITS_RIGHT"]
        self.orbit_height = (self.orbits_max-self.orbits_min)/self.NUM_ORBITS
        self.orbits = []
        for i in range(self.NUM_ORBITS):
            lower_bound = self.orbits_max-(i+1)*self.orbit_height
            upper_bound = self.orbits_max-i*self.orbit_height
            self.orbits.append([lower_bound, upper_bound])

    def get_config_state(self, request):
        if request.frame_of_reference == "eoss_config":
            return TuiStateResponse(self.blocks2bitstring(self.block_state))
            # note that orbits are currently specified in tuio space. needs to be in arm space.

    def get_orbit(self, y):
        for i, orbit in enumerate(self.orbits):
            if y > orbit[0] and y < orbit[1]:
                return i

    def blocks2bitstring(self, block_arr):
        raw_btstr = '0'*60
        for block in block_arr:
            if block["x"] < self.orbits_left or block["x"] > self.orbits_right:
                continue
            if block["y"] < self.orbits_min or block["y"] > self.orbits_max:
                continue
            id = block["id"]
            orbit = self.get_orbit(block["y"])
            index = 12*orbit + id
            raw_btstr = raw_btstr[:index] + '1' + raw_btstr[index+1:]  # b/c can't modify string
        return raw_btstr

    def shutdown_tracker(self):
        self.get_config_server.shutdown()
        TuiStateTracker.shutdown_tracker(self)
