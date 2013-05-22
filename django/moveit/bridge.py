import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject

class Planner(object):
    move_group = None
    def __init__(self):
        rospy.init_node('moveit_web')

    def goals_reachable(self):
        # Load the goals from wherever
        goals = self.load_goals()

        # Set the World (cube)
        self.load_scene()

        # Set the robot position
        # ????

        # Calculate reachability
        id = 0
        for goal in goals:
            goal.reachable = self.is_reachable(goal)
            goal.id = id
            id += 1

        # Return goals with reachability embedded
        return goals

    def load_scene():
        # TODO
        pass

    def load_goals(self):
        p = Pose()
        p.position.x = 0.585315333893
        p.position.y = -0.188
        p.position.z = 1.25001048644
        p.orientation.x = -0.0
        p.orientation.y = -0.887929895957
        p.orientation.z = -0.0
        p.orientation.w = 0.459978803714
        return [p]

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
    if not _planner:
        _planner = Planner()
    return _planner
