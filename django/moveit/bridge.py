import rospy
from moveit_ros_planning_interface import _moveit_move_group_interface
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
        self.psw_pub = rospy.Publisher('/planning_scene_world', PlanningSceneWorld)

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
        self._move_group = self.move_group._g
        self.ps = PlanningSceneInterface()

    def set_scene(self, scene):
        psw = PlanningSceneWorld()
        for co_json in scene['objects']:
            # TODO: Fix orientation by using proper quaternions on the client
            pose = self._make_pose(co_json['pose'])
            # TODO: Decide what to do with STL vs. Collada. The client has a Collada
            # loader but the PlanningSceneInterface can only deal with STL.
            # TODO: Proper mapping between filenames and URLs
            # filename = '/home/julian/aaad/moveit/src/moveit_web/django%s' % co_json['meshUrl']
            filename = '/home/julian/aaad/moveit/src/moveit_web/django/static/meshes/table_4legs.stl'
            co = self.ps.make_mesh(co_json['name'], pose, filename)
            psw.collision_objects.append(co)
        self.psw_pub.publish(psw)


    def get_link_poses(self):
        return self._move_group.get_link_poses_compressed()

    # Create link back to socket.io namespace to allow emitting information
    def set_socket(self, namespace):
        self.namespace = namespace

    def emit(self, event, data=None):
        if self.namespace:
            self.namespace.emit(event, data)

    def emit_new_goal(self, pose):
        self.emit('target_pose', message_converter.convert_ros_message_to_dictionary(pose)['pose'])

    def set_random_goal(self):
        goal_pose = self.move_group.get_random_pose()
        self.emit_new_goal(goal_pose)

    def _make_pose(self, json_pose):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pp = json_pose['position']
        pose.pose.position.x = pp['x']
        pose.pose.position.y = pp['y']
        pose.pose.position.z = pp['z']
        # TODO: Orientation is not working. See about
        # properly using Quaternions everywhere
        pp = json_pose['orientation']
        pose.pose.orientation.x = pp['x']
        pose.pose.orientation.y = pp['y']
        pose.pose.orientation.z = pp['z']
        pose.pose.orientation.w = pp['w']
        return pose

    def plan_to_poses(self, poses):
        goal_pose = self._make_pose(poses[0])
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
            # self.publish_position(trajectory, i)

            # TODO: Only say "True" to update state on the last step of the trajectory
            new_poses = self._move_group.update_robot_state(trajectory.joint_trajectory.joint_names,
                    trajectory.joint_trajectory.points[i].positions, True)
            self.emit('link_poses', new_poses)

        self.emit('status',{'text':'Ready to plan','ready':True})

    def deleteme_joint_update(self):
        print self._move_group.get_joints()

_planner = None
def get_planner():
    global _planner
    if not _planner:
        _planner = Planner()
    return _planner
