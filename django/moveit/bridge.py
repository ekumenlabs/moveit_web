import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject
from rospy_message_converter import message_converter

class Planner(object):
    move_group = None
    goals = []
    def __init__(self):
        rospy.init_node('moveit_web',disable_signals=True)

    def calculate_goals(self):
        # Load the goals from wherever
        self.load_goals()

        # Set the World (cube)
        self.load_scene()

        # Set the robot position
        # ????

        # Calculate reachability
        for goal in self.goals:
            goal['reachable'] = self.is_reachable(goal['pose'])

    def get_goals_as_json(self):
        ret = []
        for goal in self.goals:
            newgoal = goal.copy()
            newgoal['pose'] = message_converter.convert_ros_message_to_dictionary(newgoal['pose'])
            ret.append(newgoal)
        return ret

    def load_scene(self):
        # TODO
        pass

    def load_goals(self):
        p = Pose()
        p.position.x = 0.7
        p.position.y = -0.188
        p.position.z = 1.25001048644
        p.orientation.x = -0.0
        p.orientation.y = -0.887929895957
        p.orientation.z = -0.0
        p.orientation.w = 0.459978803714
        self.goals = [{'pose': p, 'reachable': 0, 'id': 0}]

    def is_reachable(self, pose):
        """
        Calculates reachability to a given pose
        Returns an integer state:
        - 0: not reachable
        - 1: reachable
        """
        self.move_group = MoveGroupCommander('right_arm_and_torso')
        self.move_group.set_pose_target(pose)
        trajectory = self.move_group.plan()

        print trajectory
        if trajectory is None:
            return 0
        else:
            return 1

_planner = None
def get_planner():
    global _planner
    if not _planner:
        _planner = Planner()
    return _planner
