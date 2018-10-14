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
from dabot.msg import CalibrateAction, CalibrateFeedback, CalibrateResult, MoveBlockAction, MoveBlockFeedback, MoveBlockResult
from dabot.msg import CalibrationParams
from dabot.srv import TuiState, TuiStateRequest
from dautils import get_ros_param, get_arm_bounds, get_tuio_bounds, tuio_to_ratio
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from kinova_msgs.msg import *
from kinova_msgs.srv import *
from moveit_commander import RobotCommander, MoveGroupCommander
import json


class DaArmServer:
    """The basic, design problem/tui agnostic arm server
    """
    gestures = {}
    grasp_height = 0.1

    def __init__(self, num_planning_attempts=10):
        rospy.init_node("daarm_server", anonymous=True)
        self.calibration_publisher = rospy.Publisher("/calibration_results", CalibrationParams)
        rospy.wait_for_service('get_tui_blocks')
        self.get_block_state = rospy.ServiceProxy('get_tui_blocks', TuiState)
        self.init_params()
        self.init_publishers()
        self.init_arm()

    def init_arm(self, num_planning_attempts=20):
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")
        self.robot = RobotCommander()
        self.arm.set_num_planning_attempts(num_planning_attempts)
        self.arm.set_goal_tolerance(0.2)
        self.init_action_servers()

    def init_params(self):
        try:
            self.grasp_height = get_ros_param("GRASP_HEIGHT", "Grasp height defaulting to 0.1")
        except ValueError as e:
            rospy.loginfo(e)

    def init_publishers(self):
        self.action_belief_publisher = rospy.Publisher("/arm_action_beliefs", String, queue_size=1)
        rospy.sleep(0.5)

    def init_action_servers(self):
        self.calibration_server = actionlib.SimpleActionServer("calibrate_arm", CalibrateAction, self.calibrate)
        self.move_block_server = actionlib.SimpleActionServer("move_block", MoveBlockAction, self.handle_move_block)

    def init_action_clients(self):
        # Action Client for joint control
        action_address = '/j2s7s300_driver/joints_action/joint_angles'
        self.jointActionClient = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)
        rospy.loginfo('Waiting for ArmJointAnglesAction server...')
        self.jointActionClient.wait_for_server()
        rospy.loginfo('ArmJointAnglesAction Server Connected')

    def init_services(self):
        # Service to get TUI State
        self.tui_state_service = rospy.ServiceProxy('/get_tui_blocks', TuiState)

        # Service for homing the arm
        home_arm_service = '/j2s7s300_driver/in/home_arm'
        self.home_arm_service = rospy.ServiceProxy(home_arm_service, HomeArm)
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

    def home_arm(self):
        # send the arm home
        pass

    def home_arm_kinova(self):
        """Takes the arm to the kinova default home if possible
        """
        self.arm.set_named_target("Home")
        self.arm.go()

    def handle_move_block(self, message):
        """msg format: {id: int,
                        source: Point {x: float,y: float},
                        target: Point {x: float, y: float}
        """

    def open_gripper(self, delay=0):
        """open the gripper ALL THE WAY, then delay
        """
        self.gripper.set_named_target("Open")
        self.gripper.go()
        rospy.sleep(delay)

    def close_gripper(self, delay=0):
        """close the gripper ALL THE WAY, then delay
        """
        self.gripper.set_named_target("Close")
        self.gripper.go()
        rospy.sleep(delay)

    def handle_gesture(self, gesture):
        # lookup the gesture from a table? or how about a message?
        # one possibility, object that contains desired deltas in pos/orientation
        # as well as specify the arm or gripper choice
        pass

    def move_arm_to_pose(self, position, orientation, delay=0, action_server=None):
        p = self.arm.get_current_pose()
        p.pose.position = position
        p.pose.orientation = orientation
        self.arm.set_pose_target(p)
        plan = self.arm.plan()
        if plan:
            if action_server:
                action_server.publish_feedback(CalibrateFeedback("Plan Found"))
            self.arm.execute(plan)
            rospy.sleep(delay)
        else:
            if action_server:
                action_server.publish_feedback(CalibrateFeedback("Planning Failed"))

    def handle_move_sequence(self, message):
        pass

    def get_grasp_orientation(self, position):
        return Quaternion(1, 0, 0, 0)

    def move_z(self, distance):
        p = self.arm.get_current_pose()
        p.pose.position.z += distance
        self.move_arm_to_pose(p.pose.position, p.pose.orientation, delay=0)

    def pick_block(self, location, check_grasp=False, retry_attempts=0, delay=0, action_server=None):
        # go to a spot and pick up a block
        # if check_grasp is true, it will check torque on gripper and retry or fail if not holding
        p = self.arm.get_current_pose()
        position = Point(location.x, location.y, self.grasp_height)
        orientation = self.get_grasp_orientation(position)
        try:
            self.move_arm_to_pose(position, orientation, delay=0, action_server=action_server)
        except Exception as e:
            raise(e)
        if action_server:
            action_server.publish_feedback()

        if check_grasp is True:
            for i in range(retry_attempts):
                self.move_z(0.3)

    def place_block(self, location, check_grasp=False):
        # go to a spot an drop a block
        # if check_grasp is true, it will check torque on gripper and fail if not holding a block

        pass

    def stop_motion(self, home=False):
        # cancel the moveit_trajectory
        self.arm.stop()
        # call the emergency stop on kinova
        self.emergency_stop()
        if home is True:
            self.home_arm()

    def calibrate(self, message):
        print("calibrating ", message)
        self.place_calibration_block()
        rospy.sleep(5)  # five seconds to correct the drop if it bounced, etc.
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
        self.move_arm_to_pose(Point(0.4, -0.4, 0.1), Quaternion(0, 1, 0, 0))
        # drop the block
        self.open_gripper()
        self.calibration_server.publish_feedback(CalibrateFeedback("dropped the block"))
        calibration_block = {'x': 0.4, 'y': 0.4, 'id': 0}
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
