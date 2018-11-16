#!/usr/bin/env python
"""
EOSS Arm Server
===================
EOSS-Specific actions (e.g. moving blocks between orbits)
"""
from daarm.daarm_server import DaArmServer
from dabot.srv import TuiState, TuiStateResponse, FindEossBlock, FindEossBlockResponse
from dautils import get_ros_param
import rospy
from std_msgs.msg import String
from dabot.msg import MoveBlockGoal, MoveEossBlockAction, MoveEossBlockResult
import actionlib

class EossArmServer(DaArmServer):
    def __init__(self):
        # initialize the parent
        DaArmServer.__init__(self, safe_zone = self.get_zone(-1))
        self.move_eoss_block_server = actionlib.SimpleActionServer(
            "move_eoss_block", MoveEossBlockAction, self.handle_EOSS_action, auto_start=False)
        self.move_eoss_block_server.start()

        self.check_move_doable_service = rospy.Service("check_move",FindEossBlock,self.check_doable)
        
        #self._result = MoveEossBlockActionResult()

    def handle_EOSS_action(self,message):
        # message is {blockid, source={-1,0,1,2,3,4}, target = {-1,0,1,2,3,4}}
        source, source_zone = self.get_zone(message.source)
        target, target_zone = self.get_zone(message.target)
        #goal = MoveBlockGoal()
        id = message.id
        source_x_tol = source_zone['x_tolerance']
        source_y_tol = source_zone['y_tolerance']
        target_x_tol = target_zone['x_tolerance']
        if message.target >= 0:
            target_y_tol = 0.0 
        else:
            target_y_tol = target_zone['y_tolerance']
        try:
            self.move_block(id,source['x'],source['y'],source_x_tol,source_y_tol,target['x'],target['y'],target_x_tol,target_y_tol)
            self.move_eoss_block_server.set_succeeded(MoveEossBlockResult("success"))
        except Exception as e:
            rospy.loginfo("Move Block from EOSS Action failed: "+str(e))    
            self.move_eoss_block_server.set_aborted(MoveEossBlockResult(str(e)))


    def check_doable(self, message):
        source, source_zone = self.get_zone(message.source)
        return FindEossBlockResponse(len(self.get_candidate_blocks(message.id,source['x'],source['y'],source_zone['x_tolerance'],source_zone['y_tolerance'])))

    def get_zone(self,zone_id):
        orbits_min = get_ros_param('ORBITS_MIN')
        table_left = get_ros_param('TUIO_X_MAX')
        table_height = get_ros_param('TUIO_Y_MAX')-get_ros_param('TUIO_Y_MIN')
        table_midpoint_y = get_ros_param('TUIO_Y_MIN') + table_height/2
        orbits_left = get_ros_param('ORBITS_LEFT')
        orbit_width = get_ros_param('ORBITS_LEFT')-get_ros_param('ORBITS_RIGHT')
        orbit_height = (get_ros_param('ORBITS_MAX')-get_ros_param('ORBITS_MIN'))/5
        orbits_midpoint_x = get_ros_param('ORBITS_LEFT')- orbit_width/2
        staging_right = orbits_left + get_ros_param('STAGING_BUFFER_WIDTH')
        staging_width = table_left - staging_right
        horizontal_scale = [1.0,1.0,1.0,1.0,1.0]
        if zone_id >= 0 and zone_id < 5:
            # return an orbit
            return [
                {
                    'x': orbits_midpoint_x*horizontal_scale[zone_id],
                    'y': orbits_min + (2*zone_id+1)*orbit_height/2
                },
                {
                    'x_tolerance': horizontal_scale[zone_id]*orbit_width/2,
                    'y_tolerance': orbit_height/2
                }
            ]
        elif zone_id == -1:
            # return staging
            return [{
                'x': table_left - staging_width/2,
                'y': table_midpoint_y
            },
            {
               'x_tolerance': staging_width/2,
                'y_tolerance': table_height/2
            }
            ]
        else:
            raise Exception("Did not get a valid zone.")
        
if __name__ == '__main__':
    e = EossArmServer()
