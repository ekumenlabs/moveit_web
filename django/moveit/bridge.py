import os.path
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneWorld
from sensor_msgs.msg import JointState
from rospy_message_converter import message_converter
import gevent


class Planner(object):
    move_group = None
    goals = None
    jspub = None
    namespace = None

    # These will eventually go to model objects
    robot_data = {
        'group_name': 'right_arm_and_torso',
        'eef_link': 'r_wrist_joint_link'
    }

    # Current state of the 'session' (right now, only one)
    current_scene = None
    status = None
    link_poses = None

    def __init__(self):
        rospy.init_node('moveit_web', disable_signals=True)
        self.jspub = rospy.Publisher('/update_joint_states', JointState)
        self.psw_pub = rospy.Publisher('/planning_scene_world',
                                       PlanningSceneWorld)

        # Give time for subscribers to connect to the publisher
        rospy.sleep(1)
        self.goals = []

        # HACK: Synthesize a valid initial joint configuration for PR2
        initial_joint_state = JointState()
        initial_joint_state.name = ['r_elbow_flex_joint']
        initial_joint_state.position = [-0.1]
        self.jspub.publish(initial_joint_state)

        # Create group we'll use all along this demo
        # self.move_group = MoveGroupCommander('right_arm_and_torso')
        self.move_group = MoveGroupCommander(self.robot_data['group_name'])
        self._move_group = self.move_group._g
        self.ps = PlanningSceneInterface()

        self.status = {'text': 'ready to plan', 'ready': True}

    def get_scene(self):
        return self.current_scene

    def set_scene(self, scene, meshes_root):
        self.current_scene = scene
        psw = PlanningSceneWorld()
        for co_json in scene['objects']:
            # TODO: Fix orientation by using proper quaternions on the client
            pose = self._make_pose(co_json['pose'])
            # TODO: what to do with STL vs. Collada? The client has a Collada
            # loader but the PlanningSceneInterface can only deal with STL.
            filename = os.path.join(meshes_root, 'table_4legs.stl')
            co = self.ps.make_mesh(co_json['name'], pose, filename)
            psw.collision_objects.append(co)
        self.psw_pub.publish(psw)

    def get_link_poses(self):
        if self.link_poses is None:
            self.link_poses = self._move_group.get_link_poses_compressed()
        return self.link_poses

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
        # goal_pose = self.move_group.get_random_pose('base_footprint')
        self.emit_new_goal(goal_pose)

    def _make_pose(self, json_pose):
        pose = PoseStamped()
        pose.header.frame_id = "odom_combined"
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
        # self.move_group.set_pose_target(goal_pose,'base_footprint')
        self.emit('status', {'text': 'Starting to plan'})
        trajectory = self.move_group.plan()
        number_of_joins = len(trajectory.joint_trajectory.joint_names)
        if trajectory is None or number_of_joins == 0:
            self.status = {
                'reachable': False,
                'text': 'Ready to plan',
                'ready': True
            }
            self.emit('status', self.status)
        else:
            self.status = {
                'reachable': True,
                'text': 'Rendering trajectory'
            }
            self.emit('status', self.status)
            self.publish_trajectory(trajectory)

    def publish_goal_position(self, trajectory):
        self.publish_position(self, trajectory, -1)

    def publish_position(self, trajectory, step):
        jsmsg = JointState()
        jsmsg.name = trajectory.joint_trajectory.joint_names
        jsmsg.position = trajectory.joint_trajectory.points[step].positions
        self.jspub.publish(jsmsg)

    def publish_trajectory(self, trajectory):
        cur_time = trajectory.joint_trajectory.points[0].time_from_start
        acceleration = 4.0
        for i in range(len(trajectory.joint_trajectory.points)):
            point = trajectory.joint_trajectory.points[i]
            wait_duration = point.time_from_start - cur_time
            wait_duration_time = wait_duration.to_sec() + wait_duration.to_nsec() * 1e-9
            gevent.sleep(wait_duration_time / acceleration)
            cur_time = point.time_from_start
            # self.publish_position(trajectory, i)

            # TODO: "True" only to update state the last step of the trajectory
            new_poses = self._move_group.update_robot_state(
                trajectory.joint_trajectory.joint_names,
                list(trajectory.joint_trajectory.points[i].positions), True)
            self.link_poses = new_poses
            self.emit('link_poses', new_poses)

        self.status = {'text': 'Ready to plan', 'ready': True}
        self.emit('status', self.status)

_planner = None
def get_planner():
    global _planner
    if not _planner:
        _planner = Planner()
    return _planner
