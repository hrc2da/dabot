#!/usr/bin/env python

"""
DaArmServer
============
This provides an interface to control the Jaco arm via an action library
Basic actions:
1) Move a block from one region to another
    n.b. the basic arm knows NOTHING about orbits/voting precincts, etc.
    all the logic of translating a config to a block layout should be done
    in either an extended version of the arm or in a separate node
2) Perform a gesture from a dict of gestures stored in ROS Param server
3) Calibrate the arm space to tuio space mapping
"""

import rospy
import actionlib
from dabot.msg import CalibrateAction, CalibrateFeedback, CalibrateResult, \
    MoveBlockAction, MoveBlockFeedback, MoveBlockResult, \
    MovePoseAction, MovePoseFeedback, MovePoseResult
from dabot.msg import CalibrationParams
from dabot.srv import TuiState, TuiStateRequest, ArmCommand, ArmCommandResponse
from dautils import get_ros_param, get_arm_bounds, get_tuio_bounds, tuio_to_ratio, tuio_to_arm
from geometry_msgs.msg import Point, Quaternion, PoseStamped, WrenchStamped
from std_msgs.msg import String
from kinova_msgs.msg import *
from kinova_msgs.srv import *
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface, MoveItCommanderException
from moveit_msgs.msg import MoveGroupActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
import json
import numpy as np
import random


class DaArmServer:
    """The basic, design problem/tui agnostic arm server
    """
    gestures = {}
    pointing_height = 0.06
    grasp_height = 0.05
    drop_height = 0.07
    cruising_height = 0.1
    START_TOLERANCE = 0.05 # this is for moveit to check for change in joint angles before moving
    GOAL_TOLERANCE = 0.005
    moving = False
    paused = False
    move_group_state = "IDLE"
    last_joint_trajectory_goal = ""
    last_joint_trajectory_result = ""

    def __init__(self, num_planning_attempts=100, safe_zone = None):
        rospy.init_node("daarm_server", anonymous=True)
        self.safe_zone = safe_zone # this is a fallback zone to drop a block on fail if nothing is passed: [{x,y},{xT,yT}]
        self.init_params()
        self.init_scene()
        self.init_publishers()
        self.init_subscribers()
        self.init_action_clients()
        self.init_service_clients()
        self.init_arm(num_planning_attempts)

    def init_arm(self, num_planning_attempts=700):
        rospy.set_param("/move_group/trajectory_execution/allowed_start_tolerance", self.START_TOLERANCE)
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")
        self.robot = RobotCommander()
        self.arm.set_num_planning_attempts(num_planning_attempts)
        self.arm.set_goal_position_tolerance(self.GOAL_TOLERANCE)
        self.arm.set_goal_orientation_tolerance(0.02)
        self.init_services()
        self.init_action_servers()

    def init_scene(self):
        world_objects = ["table", "tui", "monitor", "overHead", "wall", "farWall", "frontWall", "backWall", "blockProtector", "rearCamera"]
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        for obj in world_objects:
            self.scene.remove_world_object(obj)
        rospy.sleep(0.5)
        self.tuiPose = PoseStamped()
        self.tuiPose.header.frame_id = self.robot.get_planning_frame()
        self.tuiPose.pose.position = Point(0.0056, -0.343, -0.51)
        self.tuiDimension = (0.9906, 0.8382, 0.8836)
        self.overHeadPose = PoseStamped()
        self.overHeadPose.header.frame_id = self.robot.get_planning_frame()
        self.overHeadPose.pose.position = Point(0.0056, 0.0, 0.97)
        self.overHeadDimension = (0.9906,0.8382, 0.05)
        self.blockProtectorPose = PoseStamped()
        self.blockProtectorPose.header.frame_id = self.robot.get_planning_frame()
        self.blockProtectorPose.pose.position = Point(0.0056, -0.343, -0.51+ self.cruising_height)
        self.blockProtectorDimension = (0.9906, 0.8382, 0.8636)
        self.wallPose = PoseStamped()
        self.wallPose.header.frame_id = self.robot.get_planning_frame()
        self.wallPose.pose.position = Point(-0.858, -0.343, -0.3048)
        self.wallDimension = (0.6096, 2, 1.35)
        self.farWallPose = PoseStamped()
        self.farWallPose.header.frame_id = self.robot.get_planning_frame()
        self.farWallPose.pose.position = Point(0.9, -0.343, -0.3048)
        self.farWallDimension = (0.6096, 2, 3.35)
        self.frontWallPose = PoseStamped()
        self.frontWallPose.header.frame_id = self.robot.get_planning_frame()
        self.frontWallPose.pose.position = Point(0.0056,-0.85,-0.51)
        self.frontWallDimension = (1,0.15,4)
        self.backWallPose = PoseStamped()
        self.backWallPose.header.frame_id = self.robot.get_planning_frame()
        self.backWallPose.pose.position = Point(0.0056,0.55,-0.51)
        self.backWallDimension = (1,0.005,4)
        self.rearCameraPose = PoseStamped()
        self.rearCameraPose.header.frame_id = self.robot.get_planning_frame()
        self.rearCameraPose.pose.position = Point(0.65,0.45,-0.51)
        self.rearCameraDimension = (0.5,0.5,2)
        rospy.sleep(0.5)
        self.scene.add_box("tui", self.tuiPose, self.tuiDimension)
        self.scene.add_box("wall", self.wallPose, self.wallDimension)
        self.scene.add_box("farWall", self.farWallPose, self.farWallDimension)
        self.scene.add_box("overHead", self.overHeadPose, self.overHeadDimension)
        self.scene.add_box("backWall", self.backWallPose, self.backWallDimension)
        self.scene.add_box("frontWall", self.frontWallPose, self.frontWallDimension)
        self.scene.add_box("rearCamera", self.rearCameraPose, self.rearCameraDimension)
    def raise_table(self):
        #raises the table obstacle to protect blocks on the table during transport
        self.scene.remove_world_object("blockProtector")
        self.scene.add_box("blockProtector", self.blockProtectorPose, self.blockProtectorDimension)

    def lower_table(self):
        #lowers the table to allow grasping into it
        self.scene.remove_world_object("blockProtector")

    def init_params(self):
        try:
            self.grasp_height = get_ros_param("GRASP_HEIGHT", "Grasp height defaulting to 0.01")
            self.drop_height = get_ros_param("DROP_HEIGHT", "Drop height defaulting to 0.07")
            self.cruising_height = get_ros_param("CRUISING_HEIGHT", "Cruising height defaulting to 0.1")
            self.pointing_height = get_ros_param("POINT_HEIGHT", "Pointing height defaulting to 0.06")
        except ValueError as e:
            rospy.loginfo(e)
    def handle_param_update(self, message):
        self.init_params()

    def init_publishers(self):
        self.calibration_publisher = rospy.Publisher("/calibration_results", CalibrationParams, queue_size=1)
        self.action_belief_publisher = rospy.Publisher("/arm_action_beliefs", String, queue_size=1)
        rospy.sleep(0.5)

    def init_subscribers(self):
        self.joint_angle_subscriber = rospy.Subscriber(
            '/j2s7s300_driver/out/joint_angles', JointAngles, self.update_joints)
        
        self.joint_trajectory_subscriber = rospy.Subscriber(
            '/j2s7s300/follow_joint_trajectory/status', GoalStatusArray, self.update_joint_trajectory_state)

        self.joint_trajectory_goal_subscriber = rospy.Subscriber(
            '/j2s7s300/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, self.update_joint_trajectory_goal)

        self.joint_trajectory_result_subscriber = rospy.Subscriber(
            '/j2s7s300/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, self.update_joint_trajectory_result)

        self.finger_position_subscriber = rospy.Subscriber(
            '/j2s7s300_driver/out/finger_position', FingerPosition, self.update_finger_position)

        self.param_update_subscriber = rospy.Subscriber("/param_update", String, self.handle_param_update)

        self.moveit_status_subscriber = rospy.Subscriber(
            '/move_group/status', GoalStatusArray, self.update_move_group_status)
        self.move_it_feedback_subscriber = rospy.Subscriber(
             '/move_group/feedback', MoveGroupActionFeedback, self.update_move_group_state)

         #Topic for getting joint torques
        rospy.Subscriber('/j2s7s300_driver/out/joint_torques', JointAngles,self.monitorJointTorques)
        #Topic for getting cartesian force on end effector
        rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.monitorToolWrench)


    def init_action_servers(self):
        self.calibration_server = actionlib.SimpleActionServer(
            "calibrate_arm", CalibrateAction, self.calibrate, auto_start=False)
        self.calibration_server.start()
        self.move_block_server = actionlib.SimpleActionServer(
            "move_block", MoveBlockAction, self.handle_move_block, auto_start=False)
        self.move_block_server.start()
        #self.home_arm_server = actionlib.SimpleActionServer("home_arm", HomeArmAction, self.home_arm)
        self.move_pose_server = actionlib.SimpleActionServer(
            "move_pose", MovePoseAction, self.handle_move_pose, auto_start=False)
        self.move_pose_server.start()

    def init_services(self):
        self.home_arm_service = rospy.Service("/home_arm", ArmCommand, self.handle_home_arm)
        # emergency stop
        self.stop_arm_service = rospy.Service("/stop_arm", ArmCommand, self.handle_stop_arm)
        # stop and pause for a bit
        self.pause_arm_service = rospy.Service("/pause_arm", ArmCommand, self.handle_pause_arm)
        self.start_arm_service = rospy.Service("/restart_arm", ArmCommand, self.handle_restart_arm)

    def init_action_clients(self):
        # Action Client for joint control
        joint_action_address = '/j2s7s300_driver/joints_action/joint_angles'
        self.joint_action_client = actionlib.SimpleActionClient(
            joint_action_address, kinova_msgs.msg.ArmJointAnglesAction)
        rospy.loginfo('Waiting for ArmJointAnglesAction server...')
        self.joint_action_client.wait_for_server()
        rospy.loginfo('ArmJointAnglesAction Server Connected')

        # Service to move the gripper fingers
        finger_action_address = '/j2s7s300_driver/fingers_action/finger_positions'
        self.finger_action_client = actionlib.SimpleActionClient(
            finger_action_address, kinova_msgs.msg.SetFingersPositionAction)
        self.finger_action_client.wait_for_server()

        #

    def init_service_clients(self):
        self.is_simulation = False
        try:
            self.is_simulation = get_ros_param("IS_SIMULATION", "")
        except:
            self.is_simulation = False

        if self.is_simulation is True:
            # setup alternatives to jaco services for emergency stop, joint control, and finger control
            pass
        # Service to get TUI State
        rospy.wait_for_service('get_tui_blocks')
        self.get_block_state = rospy.ServiceProxy('get_tui_blocks', TuiState)

        # Service for homing the arm
        home_arm_service = '/j2s7s300_driver/in/home_arm'
        self.home_arm_client = rospy.ServiceProxy(home_arm_service, HomeArm)
        rospy.loginfo('Waiting for kinova home arm service')
        rospy.wait_for_service(home_arm_service)
        rospy.loginfo('Kinova home arm service server connected')

        # Service for emergency stop
        emergency_service = '/j2s7s300_driver/in/stop'
        self.emergency_stop = rospy.ServiceProxy(emergency_service, Stop)
        rospy.loginfo('Waiting for Stop service')
        rospy.wait_for_service(emergency_service)
        rospy.loginfo('Stop service server connected')

        # Service for restarting the arm
        start_service = '/j2s7s300_driver/in/start'
        self.restart_arm = rospy.ServiceProxy(start_service, Start)
        rospy.loginfo('Waiting for Start service')
        rospy.wait_for_service(start_service)
        rospy.loginfo('Start service server connected')

    def handle_start_arm(self, message):
        return self.restart_arm()

    def handle_stop_arm(self, message):
        return self.stop_motion()

    def handle_pause_arm(self, message):
        self.stop_motion(home=True, pause=True)
        return str(self.paused)

    def handle_restart_arm(self, message):
        self.restart_arm()
        self.paused = False
        return str(self.paused)

    def handle_home_arm(self, message):
        try:
            status = self.home_arm()
            return json.dumps(status)
        except rospy.ServiceException as e:
            rospy.loginfo("Homing arm failed")

    def home_arm(self):
        # send the arm home
        # for now, let's just use the kinova home
        #self.home_arm_client()
        self.home_arm_kinova()
        return "done"

    def custom_home_arm(self):
        angles_set = [629.776062012,150.076568694,-0.13603515923,29.8505859375,0.172727271914,212.423721313,539.743164062]
        goal = kinova_msgs.msg.ArmJointAnglesGoal()
        
        goal.angles.joint1 = angles_set[0]
        goal.angles.joint2 = angles_set[1]
        goal.angles.joint3 = angles_set[2]
        goal.angles.joint4 = angles_set[3]
        goal.angles.joint5 = angles_set[4]
        goal.angles.joint6 = angles_set[5]
        goal.angles.joint7 = angles_set[6]

        self.joint_action_client.send_goal(goal)

    def home_arm_kinova(self):
        """Takes the arm to the kinova default home if possible
        """
        # self.arm.set_named_target("Home")
        angles_set = map(np.deg2rad,[629.776062012,150.076568694,-0.13603515923,29.8505859375,0.172727271914,212.423721313,269.743164062])
        self.arm.clear_pose_targets()
        try:
            self.arm.set_joint_value_target(angles_set)
        except MoveItCommanderException as e:
            pass #stupid bug in movegroupcommander wrapper throws an exception when trying to set joint angles
        try:
            self.arm.go()
            return "successful home"
        except:
            return "failed to home"

    # This callback function monitors the Joint Torques and stops the current execution if the Joint Torques exceed certain value
    def monitorJointTorques(self,torques):
        if abs(torques.joint1) > 1:
            return
            #self.emergency_stop() #Stop arm driver
            #rospy.sleep(1.0)
            #self.group.stop() #Stop moveit execution

    # This callback function monitors the Joint Wrench and stops the current
    # execution if the Joint Wrench exceeds certain value
    def monitorToolWrench(self, wrenchStamped):
        return
        #toolwrench = abs(wrenchStamped.wrench.force.x**2 + wrenchStamped.wrench.force.y**2 + wrenchStamped.wrench.force.z**2)
        ##print toolwrench
        #if toolwrench > 100:
        #    self.emergency_stop()  # Stop arm driver
    

    def move_fingers(self, finger1_pct, finger2_pct, finger3_pct):
        finger_max_turn = 6800
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float((finger1_pct/100.0)*finger_max_turn)
        goal.fingers.finger2 = float((finger2_pct/100.0)*finger_max_turn)
        goal.fingers.finger3 = float((finger3_pct/100.0)*finger_max_turn)

        self.finger_action_client.send_goal(goal)
        if self.finger_action_client.wait_for_result(rospy.Duration(5.0)):
            return self.finger_action_client.get_result()
        else:
            self.finger_action_client.cancel_all_goals()
            rospy.loginfo('the gripper action timed-out')
            return None

    def move_joint_angles(self, angle_set):
        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        self.joint_action_client.send_goal(goal)
        if self.joint_action_client.wait_for_result(rospy.Duration(20.0)):
            return self.joint_action_client.get_result()
        else:
            print('        the joint angle action timed-out')
            self.joint_action_client.cancel_all_goals()
            return None

    def handle_move_block(self, message):
        """msg format: {id: int,
                        source: Point {x: float,y: float},
                        target: Point {x: float, y: float}
        """
        print(message)

        pick_x = message.source.x
        pick_y = message.source.y
        pick_x_threshold = message.source_x_tolerance
        pick_y_threshold = message.source_y_tolerance
        block_id = message.id

        place_x = message.target.x
        place_y = message.target.y
        place_x_threshold = message.target_x_tolerance
        place_y_threshold = message.target_y_tolerance
        self.move_block(block_id,pick_x,pick_y,pick_x_threshold,pick_y_threshold,place_x,place_y,place_x_threshold,place_y_threshold,message.block_size)

    def handle_pick_failure(self,exception):
        rospy.loginfo("Pick failed, going home.")
        self.open_gripper()
        self.home_arm()
        raise exception


    def handle_place_failure(self,safe_zone,block_size,exception):
        #should probably figure out if I'm holding the block first so it doesn't look weird
        #figure out how to drop the block somewhere safe
        #pass none and none if you are certain you haven't picked up a block yet
        if safe_zone is None and block_size is None:
            self.home_arm()
            raise exception
        rospy.loginfo("HANDLING PLACE FAILURE")
        block_response = json.loads(self.get_block_state('tuio').tui_state)
        current_block_state = block_response
        drop_location = self.calculate_drop_location(
            safe_zone[0]['x'], safe_zone[0]['y'], safe_zone[1]['x_tolerance'], safe_zone[1]['y_tolerance'], current_block_state, block_size, num_attempts=1000)
        try:
            arm_drop_location = tuio_to_arm(drop_location.x, drop_location.y, get_tuio_bounds(), get_arm_bounds())
            rospy.loginfo("panic arm drop: "+str(arm_drop_location))
            self.place_block(Point(arm_drop_location[0], arm_drop_location[1], 0))
        except Exception as e:
            rospy.loginfo("ERROR: Cannot panic place the block! Get ready to catch it!")
            self.open_gripper()
        self.home_arm()
        raise exception

    def get_candidate_blocks(self,block_id,pick_x,pick_y,pick_x_tolerance,pick_y_tolerance):
        block_response = json.loads(self.get_block_state('tuio').tui_state)
        current_block_state = block_response

        candidate_blocks = []
        print("looking for ",block_id," ",pick_x,pick_y,pick_x_tolerance,pick_y_tolerance)
        # get candidate blocks -- blocks with the same id and within the error bounds/threshold given
        print(current_block_state)
        for block in current_block_state:
            print(block, self.check_block_bounds(block['x'], block['y'], pick_x, pick_y, pick_x_tolerance, pick_y_tolerance))
            if block['id'] == block_id and self.check_block_bounds(block['x'], block['y'], pick_x, pick_y, pick_x_tolerance, pick_y_tolerance):
                candidate_blocks.append(block)
        return candidate_blocks

    def move_block(self,block_id,pick_x,pick_y,pick_x_tolerance,pick_y_tolerance,place_x,place_y,
        place_x_tolerance,place_y_tolerance,block_size = None, safe_zone = None, pick_tries=2, place_tries=1, point_at_block = True):
        
        if block_size is None:
            block_size = get_ros_param('DEFAULT_BLOCK_SIZE')
        block_response = json.loads(self.get_block_state('tuio').tui_state)
        current_block_state = block_response


        candidate_blocks = []
        print("looking for ",block_id," ",pick_x,pick_y,pick_x_tolerance,pick_y_tolerance)
        # get candidate blocks -- blocks with the same id and within the error bounds/threshold given
        print(current_block_state)
        # check for a drop location before trying to pick, do this before checking source to prevent cases where we go for a block user 
        # moved while we are checking for a drop location
        drop_location = self.calculate_drop_location(
            place_x, place_y, place_x_tolerance, place_y_tolerance, current_block_state, block_size, num_attempts=2000)

        if drop_location == None:
             self.handle_place_failure(None,None,Exception("no room in the target zone to drop the block"))

        # here we are actually building a set of candidate blocks to pick
        for block in current_block_state:
            print(block, self.check_block_bounds(block['x'], block['y'], pick_x, pick_y, pick_x_tolerance, pick_y_tolerance))
            if block['id'] == block_id and self.check_block_bounds(block['x'], block['y'], pick_x, pick_y, pick_x_tolerance, pick_y_tolerance):
                candidate_blocks.append(block)

        # select best block to pick and pick up
        pick_location = None
        if len(candidate_blocks) < 1:
            raise Exception("no block of id "+str(block_id)+" found within the source zone")
        
        elif len(candidate_blocks) == 1:
            pick_location = Point(candidate_blocks[0]['x'], candidate_blocks[0]['y'],0)
        else:
            pick_location = Point(*self.find_most_isolated_block(candidate_blocks, current_block_state))

        if point_at_block == True:
            try:
                arm_pick_location = tuio_to_arm(pick_location.x, pick_location.y, get_tuio_bounds(), get_arm_bounds())
                arm_drop_location = tuio_to_arm(drop_location.x, drop_location.y, get_tuio_bounds(), get_arm_bounds())
                self.close_gripper()
                self.point_at_block(location=Point(arm_pick_location[0], arm_pick_location[1], 0))
                self.point_at_block(location=Point(arm_drop_location[0], arm_drop_location[1], 0))
                self.home_arm()
            except Exception as e:
                rospy.loginfo(str(e))
                rospy.loginfo("failed trying to point at block. giving up.")
                self.home_arm()
            self.move_block_server.set_succeeded(MoveBlockResult(drop_location))
            return
        
        for i in range(pick_tries):
            try:
                arm_pick_location = tuio_to_arm(pick_location.x, pick_location.y, get_tuio_bounds(), get_arm_bounds())
                self.pick_block(location=Point(arm_pick_location[0], arm_pick_location[1], 0), check_grasp=True)
                break
            except Exception as e:
                if i < pick_tries-1:
                    rospy.loginfo("pick failed and trying again..."+str(e))
                else:
                    rospy.loginfo("pick failed and out of attempts..."+str(e))
                    self.handle_pick_failure(e)
                    
        if safe_zone == None:
            if self.safe_zone == None:
                safe_zone = [{'x':pick_x,'y':pick_y},{'x_tolerance':pick_x_tolerance,'y_tolerance':pick_y_tolerance}]
            else:
                safe_zone = self.safe_zone
        
        # calculate drop location
        
        block_response = json.loads(self.get_block_state('tuio').tui_state)
        current_block_state = block_response
        drop_location = self.calculate_drop_location(
            place_x, place_y, place_x_tolerance, place_y_tolerance, current_block_state, block_size, num_attempts=2000)
        if drop_location == None:
            self.handle_place_failure(safe_zone, block_size, Exception("no room in the target zone to drop the block"))
        rospy.loginfo("tuio drop"+str(drop_location))
        for i in range(place_tries):
            try:
                arm_drop_location = tuio_to_arm(drop_location.x, drop_location.y, get_tuio_bounds(), get_arm_bounds())
                rospy.loginfo("arm drop: "+str(arm_drop_location))
                self.place_block(Point(arm_drop_location[0], arm_drop_location[1], 0))
                break
            except Exception as e:
                if i < place_tries-1:
                    rospy.loginfo("place failed and trying again..."+str(e))
                else:
                    rospy.loginfo("place failed and out of attempts..."+str(e))
                    # check to see if we've defined a safe zone to drop the blocks
                    
                    self.handle_place_failure(safe_zone,block_size,e)
                    
        # assume success if we made it this far
        self.move_block_server.set_succeeded(MoveBlockResult(drop_location))
        

    # check if a certain x, y position is within the bounds of another x,y position
    @staticmethod
    def check_block_bounds(x, y, x_origin, y_origin, x_threshold, y_threshold):
        if x <= x_origin + x_threshold and x >= x_origin - x_threshold \
                and y <= y_origin + y_threshold and y >= y_origin - y_threshold:
            return True
        return False

    # calculate the best location to drop the block
    def calculate_drop_location(self, x, y, x_threshold, y_threshold, blocks, block_size, num_attempts=1000):
        attempt = 0
        x_original, y_original = x, y
        while(attempt < num_attempts):
            valid = True
            for block in blocks:
                if self.check_block_bounds(block['x'], block['y'], x, y, 0.8*block_size, block_size):
                    valid = False
                    break
            if valid:
                return Point(x, y, 0)
            else:
                x = random.uniform(x_original - x_threshold, x_original + x_threshold)
                y = random.uniform(y_original - y_threshold, y_original + y_threshold)
            attempt += 1
        #if none found in num_attempts, return none
        return None

    # candidates should have more than one element in it
    @staticmethod
    def find_most_isolated_block(candidates, all_blocks):
        print(candidates)
        min_distances = []  # tuples of candidate, distance to closest block
        for candidate in candidates:
            min_dist = -1
            for block in all_blocks:
                if block['x'] == candidate['x'] and block['y'] == candidate['y']:
                    continue
                else:
                    dist = DaArmServer.block_dist(candidate, block)
                    if min_dist == -1 or dist < min_dist:
                        min_dist = dist
            min_distances.append([candidate, min_dist])
        most_isolated, _ = max(min_distances, key=lambda x: x[1])  # get most isolated candidate, and min_distance
        return most_isolated['x'], most_isolated['y'], 0

    @staticmethod
    def block_dist(block_1, block_2):
        return np.sqrt((block_2['x'] - block_1['x'])**2 + (block_2['y'] - block_1['y'])**2)

    def update_finger_position(self,message):
        self.finger_positions = [message.finger1,message.finger2,message.finger3]

    def check_grasp(self):
        closed_pos = 0.95*6800
        distance_from_closed = 0
        for fp in self.finger_positions:
            distance_from_closed += (closed_pos-fp)**2
        if np.sqrt(distance_from_closed) > 130: #this is just some magic number for now
            return True #if the fingers aren't fully closed, then grasp is good
        else:
            return False

    def open_gripper(self, delay=0):
        """open the gripper ALL THE WAY, then delay
        """
        if self.is_simulation is True:
            self.gripper.set_named_target("Open")
            self.gripper.go()
        else:
            try:
                rospy.loginfo("Opening Gripper!!")
                self.move_fingers(50, 50, 50)
            except Exception as e:
                rospy.loginfo("Caught it!!"+str(e))
        rospy.sleep(delay)

    def close_gripper(self, delay=0):
        """close the gripper ALL THE WAY, then delay
        """
        # self.gripper.set_named_target("Close")
        # self.gripper.go()
        try:
            rospy.loginfo("Closing Gripper!!")
            self.move_fingers(95, 95, 95)
        except Exception as e:
            rospy.loginfo("Caught it!!"+str(e))
        rospy.sleep(delay)

    def handle_gesture(self, gesture):
        # lookup the gesture from a table? or how about a message?
        # one possibility, object that contains desired deltas in pos/orientation
        # as well as specify the arm or gripper choice
        pass

    def handle_move_pose(self, message):
        # takes a geometry_msgs/Pose message
        self.move_arm_to_pose(message.target.position, message.target.orientation, action_server=self.move_pose_server)
        self.move_pose_server.set_succeeded()

    def check_plan_result(self,target_pose,cur_pose):
        #we'll do a very lenient check, this is to find failures, not error
        #also only checking position, not orientation
        rospy.loginfo("checking pose:"+str(target_pose)+str(cur_pose))
        if np.abs(target_pose.pose.position.x - cur_pose.pose.position.x) > self.GOAL_TOLERANCE*2:
            print("x error too far")
            return False
        if np.abs(target_pose.pose.position.y - cur_pose.pose.position.y) > self.GOAL_TOLERANCE*2:
            print("y error too far")
            return False
        if np.abs(target_pose.pose.position.z - cur_pose.pose.position.z) > self.GOAL_TOLERANCE*2:
            print("z error too far")
            return False
        print("tolerable error")
        return True
    # expects cooridinates for position to be in arm space
    def move_arm_to_pose(self, position, orientation, delay=0.5, waypoints=[], corrections=4, action_server=None):
        for i in range(corrections+1):
            rospy.loginfo("TRY NUMBER "+str(i))
            if len(waypoints) > 0 and i < 1:
                # this is good for doing gestures
                plan, fraction = self.arm.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.0)
            else:
                p = self.arm.get_current_pose()
                p.pose.position = position
                p.pose.orientation = orientation
                rospy.loginfo("PLANNING TO "+str(p))
                self.arm.set_pose_target(p)
                last_traj_goal = self.last_joint_trajectory_goal
                rospy.loginfo("EXECUTING!")
                plan = self.arm.go(wait = False)
                timeout = 5/0.001
                while self.last_joint_trajectory_goal == last_traj_goal:
                    rospy.sleep(0.001)
                    timeout -= 1
                    if timeout <= 0:
                        raise(Exception("Timeout waiting for kinova to accept movement goal."))
                rospy.loginfo("KINOVA GOT THE MOVEMENT GOAL")
                current_goal = self.last_joint_trajectory_goal
                # then loop until a result for it gets published
                timeout = 90/0.001
                while self.last_joint_trajectory_result != current_goal:
                    rospy.sleep(0.001)
                    timeout -=1
                    if timeout <= 0:
                        raise(Exception("Motion took longer than 90 seconds. aborting..."))
                    if self.paused is True:
                        self.arm.stop()  # cancel the moveit goals
                        return
                rospy.loginfo("LEAVING THE WHILE LOOP")
                # for i in range(corrections):
                #     rospy.loginfo("Correcting the pose")
                #     self.move_joint_angles(angle_set)
                rospy.sleep(delay)
                if(self.check_plan_result(p, self.arm.get_current_pose())):
                    break #we're there, no need to retry
                #rospy.loginfo("OK GOT THROUGH THE PLANNING PHASE")
            if False:
                # # get the last pose to correct if desired
                # ptPos = plan.joint_trajectory.points[-1].positions
                # # print "=================================="
                # # print "Last point of the current trajectory: "
                # angle_set = list()
                # for i in range(len(ptPos)):
                #     tempPos = ptPos[i]*180/np.pi + int(round((self.joint_angles[i] - ptPos[i]*180/np.pi)/(360)))*360
                #     angle_set.append(tempPos)

                if action_server:
                    pass
                    #action_server.publish_feedback(CalibrateFeedback("Plan Found"))
                last_traj_goal = self.last_joint_trajectory_goal
                rospy.loginfo("EXECUTING!")
                self.arm.execute(plan, wait=False)
                # this is a bit naive, but I'm going to loop until a new trajectory goal gets published
                timeout = 5/0.001
                while self.last_joint_trajectory_goal == last_traj_goal:
                    rospy.sleep(0.001)
                    timeout -= 1
                    if timeout <= 0:
                        raise(Exception("Timeout waiting for kinova to accept movement goal."))
                rospy.loginfo("KINOVA GOT THE MOVEMENT GOAL")
                current_goal = self.last_joint_trajectory_goal
                # then loop until a result for it gets published
                timeout = 15/0.001
                while self.last_joint_trajectory_result != current_goal:
                    rospy.sleep(0.001)
                    timeout -=1
                    if timeout <= 0:
                        raise(Exception("Motion took longer than 15 seconds. aborting..."))
                    if self.paused is True:
                        self.arm.stop()  # cancel the moveit goals
                        return
                rospy.loginfo("LEAVING THE WHILE LOOP")
                # for i in range(corrections):
                #     rospy.loginfo("Correcting the pose")
                #     self.move_joint_angles(angle_set)
                rospy.sleep(delay)
            else:
                if action_server:
                    #action_server.publish_feedback(CalibrateFeedback("Planning Failed"))
                    pass

    # Let's have the caller handle sequences instead.
    # def handle_move_sequence(self, message):
    #     # if the move fails, do we cancel the sequence
    #     cancellable = message.cancellable
    #     moves = message.moves
    #     for move in moves:

    #         try:
    #             self.handle_move_block(move)
    #         except Exception:
    #             if cancellable:
    #                 rospy.loginfo("Part of move failed, cancelling the rest.")
    #                 break

    def update_move_group_state(self, message):
        rospy.loginfo(message.feedback.state)
        self.move_group_state = message.feedback.state
    
    def update_move_group_status(self, message):
        if message.status_list:
            #rospy.loginfo("MoveGroup Status for "+str(message.status_list[0].goal_id.id)+": "+str(message.status_list[0].status))
            self.move_group_status = message.status_list[0].status

    def update_joint_trajectory_state(self, message):
        # print(message.status_list)
        if len(message.status_list) > 0:
            self.joint_trajectory_state = message.status_list[0].status
        else:
            self.joint_trajectory_state = 0

    def update_joint_trajectory_goal(self, message):
        #print("goalisasis", message.goal_id.id)
        self.last_joint_trajectory_goal = message.goal_id.id

    def update_joint_trajectory_result(self, message):
        #print("resultisasis", message.status.goal_id.id)
        self.last_joint_trajectory_result = message.status.goal_id.id

    def get_grasp_orientation(self, position):
        #return Quaternion(0, 0, 1/np.sqrt(2), 1/np.sqrt(2))
        return Quaternion(-0.707388, -0.706825, 0.0005629, 0.0005633)

    def update_joints(self, joints):
        self.joint_angles = [joints.joint1, joints.joint2, joints.joint3,
                             joints.joint4, joints.joint5, joints.joint6, joints.joint7]

    def move_z_relative(self, distance):
        p = self.arm.get_current_pose()
        p.pose.position.z += distance
        self.move_arm_to_pose(p.pose.position, p.pose.orientation, delay=0)
    
    def move_z_absolute(self, height):
        p = self.arm.get_current_pose()
        p.pose.position.z = height
        self.move_arm_to_pose(p.pose.position, p.pose.orientation, delay=0)

    def pick_block(self, location, check_grasp=False, retry_attempts=0, delay=0, action_server=None):
        # go to a spot and pick up a block
        # if check_grasp is true, it will check torque on gripper and retry or fail if not holding
        # open the gripper
        # print('Position: ', position)
        self.open_gripper()
        position = Point(location.x, location.y, self.cruising_height)
        orientation = self.get_grasp_orientation(position)
        try:
            self.raise_table()
            self.move_arm_to_pose(position, orientation, delay=0, action_server=action_server)
            self.lower_table()
            position = Point(location.x, location.y, self.grasp_height)
            self.move_arm_to_pose(position, orientation, delay=0, action_server=action_server)
        except Exception as e:
            current_pose = self.arm.get_current_pose()
            if current_pose.pose.position.z - self.cruising_height < 0:
                self.move_z_absolute(self.crusing_height)
            self.raise_table()
            raise(e) #problem because sometimes we get exception e.g. if we're already there
            # and it will skip the close if so.
        if action_server:
            action_server.publish_feedback()
        
        self.close_gripper()
        self.move_z_absolute(self.cruising_height)
        #try to wait until we're up to check grasp
        rospy.sleep(0.5)

        if check_grasp is True:
            if(self.check_grasp() is False):
                print("Grasp failed!") 
                self.raise_table()
                raise(Exception("grasp failed!"))
            # for now, but we want to check finger torques
            # for i in range(retry_attempts):
            #     self.move_z(0.3)
        self.raise_table()
        
        rospy.sleep(delay)

    def place_block(self, location, check_grasp=False, delay=0, action_server=None):
        # go to a spot an drop a block
        # if check_grasp is true, it will check torque on gripper and fail if not holding a block
        # print('Position: ', position)
        current_pose = self.arm.get_current_pose()
        if current_pose.pose.position.z - self.cruising_height < -.02: # if I'm significantly below cruisng height, correct
            self.move_z_absolute(self.cruising_height)
        position = Point(location.x, location.y, self.cruising_height)
        rospy.loginfo("PLACE POSITION: "+str(position)+"(DROP HEIGHT: "+str(self.drop_height))
        orientation = self.get_grasp_orientation(position)
        try:
            self.raise_table() # only do this as a check in case it isn't already raised
            self.move_arm_to_pose(position, orientation, delay=0, action_server=action_server)
            self.lower_table()
            self.move_z_absolute(self.drop_height)
        except Exception as e:
            current_pose = self.arm.get_current_pose()
            if current_pose.pose.position.z - self.cruising_height < 0:
                self.move_z_absolute(self.cruising_height)
            self.raise_table()
            raise(e)
        if action_server:
            action_server.publish_feedback()
        self.open_gripper()
        self.move_z_absolute(self.cruising_height)
        self.raise_table()
        self.close_gripper()

    def point_at_block(self, location, delay=0, action_server=None):
        # go to a spot an drop a block
        # if check_grasp is true, it will check torque on gripper and fail if not holding a block
        # print('Position: ', position)
        current_pose = self.arm.get_current_pose()
        if current_pose.pose.position.z - self.cruising_height < -.02: # if I'm significantly below cruisng height, correct
            self.move_z_absolute(self.cruising_height)
        position = Point(location.x, location.y, self.cruising_height)
        rospy.loginfo("PLACE POSITION: "+str(position)+"(DROP HEIGHT: "+str(self.drop_height))
        orientation = self.get_grasp_orientation(position)
        try:
            self.raise_table() # only do this as a check in case it isn't already raised
            self.move_arm_to_pose(position, orientation, delay=0, action_server=action_server)
            self.lower_table()
            self.move_z_absolute(self.pointing_height)
            self.move_z_absolute(self.cruising_height)
        except Exception as e:
            current_pose = self.arm.get_current_pose()
            if current_pose.pose.position.z - self.cruising_height < 0:
                self.move_z_absolute(self.cruising_height)
            self.raise_table()
            raise(e)
        if action_server:
            action_server.publish_feedback()
        self.raise_table()
        
    def stop_motion(self, home=False, pause=False):
        rospy.loginfo("STOPPING the ARM")
        # cancel the moveit_trajectory
        # self.arm.stop()

        # call the emergency stop on kinova
        self.emergency_stop()
        # rospy.sleep(0.5)

        if pause is True:
            self.paused = True
        if home is True:
            # self.restart_arm()
            self.home_arm()

    def calibrate(self, message):
        print("calibrating ", message)
        self.place_calibration_block()
        rospy.sleep(5)  # five seconds to correct the drop if it bounced, etc.
        print("moving on...")
        self.calibration_server.publish_feedback(CalibrateFeedback("getting the coordinates"))
        params = self.record_calibration_params()
        self.set_arm_calibration_params(params[0], params[1])
        self.calibration_publisher.publish(CalibrationParams(params[0], params[1]))
        self.calibration_server.set_succeeded(CalibrateResult(params))

    def set_arm_calibration_params(self, arm_x_min, arm_y_min):
        rospy.set_param("ARM_X_MIN", arm_x_min)
        rospy.set_param("ARM_Y_MIN", arm_y_min)

    def place_calibration_block(self):
        # start by homing the arm (kinova home)
        self.calibration_server.publish_feedback(CalibrateFeedback("homing"))
        self.home_arm_kinova()
        # go to grab a block from a human
        self.open_gripper()
        self.close_gripper()
        self.open_gripper(4)
        self.close_gripper(2)
        # move to a predetermined spot
        self.calibration_server.publish_feedback(CalibrateFeedback("moving to drop"))
        try:
            self.move_arm_to_pose(Point(0.4, -0.4, 0.1), Quaternion(1, 0, 0, 0), corrections=0)
        except Exception as e:
            rospy.loginfo("THIS IS TH PRoblem"+str(e))
        # drop the block
        self.open_gripper()
        self.calibration_server.publish_feedback(CalibrateFeedback("dropped the block"))
        calibration_block = {'x': 0.4, 'y': -0.4, 'id': 0}
        calibration_action_belief = {"action": "add", "block": calibration_block}
        self.action_belief_publisher.publish(String(json.dumps(calibration_action_belief)))
        rospy.loginfo("published arm belief")
        return

    def record_calibration_params(self):
        """ Call the block_tracker service to get the current table config.
            Find the calibration block and set the appropriate params.
        """
        # make sure we know the dimensions of the table before we start
        # fixed table dimensions include tuio_min_x,tuio_min_y,tuio_dist_x,tuio_dist_y,arm_dist_x,arm_dist_y
        tuio_bounds = get_tuio_bounds()
        arm_bounds = get_arm_bounds(calibrate=False)
        try:
            block_state = json.loads(self.get_block_state("tuio").tui_state)
        except rospy.ServiceException as e:
            # self.calibration_server.set_aborted()
            raise(ValueError("Failed getting block state to calibrate: "+str(e)))
        if len(block_state) != 1:
            # self.calibration_server.set_aborted()
            raise(ValueError("Failed calibration, either couldn't find block or > 1 block on TUI!"))

        # if we made it here, let's continue!
        # start by comparing the reported tuio coords where we dropped the block
        # with the arm's localization of the end-effector that dropped it
        # (we assume that the block fell straight below the drop point)
        drop_pose = self.arm.get_current_pose()
        end_effector_x = drop_pose.pose.position.x
        end_effector_y = drop_pose.pose.position.y

        block_tuio_x = block_state[0]['x']
        block_tuio_y = block_state[0]['y']

        x_ratio, y_ratio = tuio_to_ratio(block_tuio_x, block_tuio_y, tuio_bounds)
        arm_x_min = end_effector_x - x_ratio*arm_bounds["x_dist"]
        arm_y_min = end_effector_y - y_ratio*arm_bounds["y_dist"]

        return [arm_x_min, arm_y_min]


if __name__ == '__main__':
    arm = DaArmServer()
    rospy.spin()
