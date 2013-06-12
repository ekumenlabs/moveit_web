import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject
from sensor_msgs.msg import JointState
from rospy_message_converter import message_converter
import json
import gevent

class Planner(object):
    move_group = None
    goals = None
    jspub = None
    namespace = None

    def __init__(self):
        rospy.init_node('moveit_web',disable_signals=True)
        self.jspub = rospy.Publisher('/update_joint_states',JointState)
        # Give time for subscribers to connect to the publisher
        rospy.sleep(1)
        self.goals = []

        # HACK: Synthesize a valid initial joint configuration for PR2
        initial_joint_state = JointState()
        initial_joint_state.name = ['r_elbow_flex_joint']
        initial_joint_state.position = [-0.1]
        self.jspub.publish(initial_joint_state)

        # Create group we'll use all along this demo
        self.move_group = MoveGroupCommander('right_arm_and_torso')

    def get_link_poses(self):
        return self.move_group._g.get_link_poses_compressed()

    # Create link back to socket.io namespace to allow emitting information
    def set_socket(self, namespace):
        self.namespace = namespace

    def emit(self, event, data=None):
        if self.namespace:
            self.namespace.emit(event, data)

    def emit_new_goal(self, pose):
        self.emit('target_pose', message_converter.convert_ros_message_to_dictionary(pose)['pose']['position'])

    def plan_to_random_goal(self):
        goal_pose = self.move_group.get_random_pose()
        self.emit_new_goal(goal_pose)
        self.move_group.set_pose_target(goal_pose)
        self.emit('status',{'text':'Starting to plan'})
        trajectory = self.move_group.plan()
        if trajectory is None or len(trajectory.joint_trajectory.joint_names) == 0:
            self.emit('status',{'reachable':False,'text':'Ready to plan','ready':True})
        else:
            self.emit('status',{'reachable':True,'text':'Rendering trajectory'})
            self.publish_trajectory(trajectory)

    def publish_goal_position(self, trajectory):
        self.publish_position(self, trajectory, -1)

    def publish_position(self, trajectory, step):
        jsmsg = JointState()
        jsmsg.name = trajectory.joint_trajectory.joint_names
        jsmsg.position = trajectory.joint_trajectory.points[step].positions
        self.jspub.publish(jsmsg)

    def publish_trajectory(self, trajectory):
        cur_time = 0.0
        acceleration = 4.0
        for i in range(len(trajectory.joint_trajectory.points)):
            point = trajectory.joint_trajectory.points[i]
            gevent.sleep((point.time_from_start - cur_time)/acceleration)
            cur_time = point.time_from_start
            self.publish_position(trajectory, i)
            self.emit('link_poses', self.get_link_poses())
        self.emit('status',{'text':'Ready to plan','ready':True})

_planner = None
def get_planner():
    global _planner
    if not _planner:
        _planner = Planner()
    return _planner
