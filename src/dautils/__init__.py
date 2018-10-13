import rospy
import actionlib
from dabot.msg import CalibrateAction, CalibrateGoal


def get_ros_param(param, err_msg):
    if(rospy.has_param(param)):
        return rospy.get_param(param)
    else:
        raise ValueError(err_msg)


def get_tuio_bounds():
    tuio_bounds = {}
    tuio_bounds['min_x'] = get_ros_param('TUIO_MIN_X', 'Tuio bounds must be configured')
    tuio_bounds['max_x'] = get_ros_param('TUIO_MAX_X', 'Tuio bounds must be configured')
    tuio_bounds['min_y'] = get_ros_param('TUIO_MIN_Y', 'Tuio bounds must be configured')
    tuio_bounds['max_y'] = get_ros_param('TUIO_MAX_Y', 'Tuio bounds must be configured')
    tuio_bounds['x_dist'] = tuio_bounds['max_x'] - tuio_bounds['min_x']
    tuio_bounds['y_dist'] = tuio_bounds['max_y'] - tuio_bounds['min_y']
    return tuio_bounds


def get_arm_bounds(calibrate=False):
    arm_bounds = {}
    arm_bounds['x_dist'] = get_ros_param('ARM_X_DIST', 'Arm reachable range must be configured')
    arm_bounds['y_dist'] = get_ros_param('ARM_Y_DIST', 'Arm reachable range must be configured')

    if rospy.has_param('ARM_X_MIN') and rospy.has_param('ARM_Y_MIN'):
        arm_bounds['x_min'] = rospy.get_param('ARM_X_MIN')
        arm_bounds['y_min'] = rospy.get_param('ARM_Y_MIN')
    elif calibrate is True:
        try:
            result = calibrate_arm()
            if(result.calibration_params):
                arm_bounds['x_min'], arm_bounds['y_min'] = result.calibration_params
            else:
                raise(ValueError("calibration failed, no params returned"))
        except ValueError as e:
            raise(e)
    else:
        return arm_bounds
    arm_bounds['x_max'] = arm_bounds['x_min'] + arm_bounds['x_dist']
    arm_bounds['y_max'] = arm_bounds['y_min'] + arm_bounds['y_dist']
    return arm_bounds


def calibrate_arm():
    client = actionlib.SimpleActionClient('calibrate_arm', CalibrateAction)
    client.wait_for_server()
    goal = CalibrateGoal("calibrate_arm")
    client.send_goal(goal, done_cb=calibrate_arm_finished,
                     active_cb=calibrate_arm_started, feedback_cb=calibrate_arm_status)
    client.wait_for_result()
    return client.get_result()


def calibrate_arm_finished(status, result):
    rospy.loginfo(status)


def calibrate_arm_started():
    rospy.loginfo("calibrate arm action requested")


def calibrate_arm_status(feedback):
    rospy.loginfo(feedback)


def tuio_to_ratio(x, y, tuio_bounds):
    # scales block coords from table's observable range into 0 to 1 scalar
    return ((tuio_bounds['max_x']-x)/tuio_bounds['x_dist'], (tuio_bounds['max_y']-y)/tuio_bounds['y_dist'])


def arm_to_ratio(x, y, arm_bounds):
    # scales block coords from arm coords to 0 to 1 scalar
    return ((arm_bounds['max_x']-x)/arm_bounds['x_dist'], (arm_bounds['max_y']-y)/arm_bounds['y_dist'])


def tuio_to_arm(x, y, tuio_bounds, arm_bounds):
    # converts block coords from tuio to arm space
    x_ratio, y_ratio = tuio_to_ratio(x, y, tuio_bounds)
    return (arm_bounds['min_x']+x_ratio*arm_bounds['x_dist'], arm_bounds['min_y']+y_ratio*arm_bounds['y_dist'])


def arm_to_tuio(x, y, tuio_bounds, arm_bounds):
    # converts block coords from tuio to arm space
    x_ratio, y_ratio = arm_to_ratio(x, y, arm_bounds)
    return (tuio_bounds['min_x']+x_ratio*tuio_bounds['x_dist'], tuio_bounds['min_y']+y_ratio*tuio_bounds['y_dist'])
