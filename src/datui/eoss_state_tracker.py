#!/usr/bin/env python
"""
EOSS State Tracker
===================
Maintains the current state of the TUI
including in terms of EOSS configuration
"""
from datui.tui_state_tracker import TuiStateTracker
from dabot.srv import TuiState, TuiStateResponse
from dautils import get_ros_param
import rospy
from std_msgs.msg import String


class EossStateTracker(TuiStateTracker):
    """
    """
    NUM_ORBITS = 5
    orbits = []

    def __init__(self):
        self.set_workspace_bounds()
        # initialize the parent
        TuiStateTracker.__init__(self, node_name="tui_state")
        self.get_config_server = rospy.Service("get_config_state", TuiState, self.get_config_state)
        self.config_publisher = rospy.Publisher("/tui_state_configs", String, queue_size=1)
        self.block_update_subscriber = rospy.Subscriber("/blocks", String, self.publish_config)
        self.param_update_subscriber = rospy.Subscriber("/param_update", String, self.handle_param_update)

    def set_workspace_bounds(self, bounds=None):
        if bounds is not None:
            self.orbits_min = bounds["ORBITS_MIN"]
            self.orbits_max = bounds["ORBITS_MAX"]
            self.orbits_left = bounds["ORBITS_LEFT"]
            self.orbits_right = bounds["ORBITS_RIGHT"]
        else:
            self.orbits_min = get_ros_param("ORBITS_MIN", "Orbit boundaries must be configured.")
            self.orbits_max = get_ros_param("ORBITS_MAX", "Orbit boundaries must be configured.")
            self.orbits_left = get_ros_param("ORBITS_LEFT", "Orbit boundaries must be configured.")
            self.orbits_right = get_ros_param("ORBITS_RIGHT", "Orbit boundaries must be configured.")

        self.orbit_height = (self.orbits_max-self.orbits_min)/self.NUM_ORBITS
        self.orbits = []
        for i in range(self.NUM_ORBITS):
            #lower_bound = self.orbits_max-(i+1)*self.orbit_height
            lower_bound = self.orbits_min+i*self.orbit_height
            #upper_bound = self.orbits_max-i*self.orbit_height
            upper_bound = self.orbits_min+(i+1)*self.orbit_height
            self.orbits.append([lower_bound, upper_bound])

    def handle_param_update(self, message):
        self.set_workspace_bounds()

    def get_config_state(self, request):
        if request.frame_of_reference == "eoss_config":
            return TuiStateResponse(self.blocks2bitstring(self.block_state))
            # note that orbits are currently specified in tuio space. needs to be in arm space.

    def get_orbit(self, y):
        for i, orbit in enumerate(self.orbits):
            if y > orbit[0] and y < orbit[1]:
                return i
        return None

    def blocks2bitstring(self, block_arr):
        raw_btstr = '0'*60
        for block in block_arr:
            if block["x"] > self.orbits_left or block["x"] < self.orbits_right:
                continue
            if block["y"] < self.orbits_min or block["y"] > self.orbits_max:
                continue
            id = block["id"]
            orbit = self.get_orbit(block["y"])
            if(orbit is not None):
                index = 12*orbit + id
                raw_btstr = raw_btstr[:index] + '1' + raw_btstr[index+1:]  # b/c can't modify string
        return raw_btstr

    def publish_config(self, message):
        # the point of this is to allow us to compare with /configs (which may have local search points as well)
        rospy.sleep(0.05)
        self.config_publisher.publish(String(self.blocks2bitstring(self.block_state)))

    def shutdown_tracker(self):
        self.get_config_server.shutdown()
        TuiStateTracker.shutdown_tracker(self)


if __name__ == '__main__':
    e = EossStateTracker()
    rospy.spin()
