import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject

rospy.init_node('move_web')
mg = MoveGroupCommander('right_arm_and_torso')

p = mg.get_current_pose()
print "Start pose:"
print p

p.pose.position.x += 0.3

#ps = PlanningSceneInterface()
#psw_pub = rospy.Publisher('/planning_scene_world', PlanningSceneWorld)
#rospy.sleep(1)

#co = ps.make_sphere("test_co", p, 0.02)
#psw = PlanningSceneWorld()
#psw.collision_objects.append(co)

#psw_pub.publish(psw)

# ps.remove_world_object("test_sphere")

# ps.add_sphere("test_sphere", p, 0.1)
# rospy.sleep(1)
# ps.add_sphere("test_sphere", p, 0.1)

#p.pose.position.x += 0.3
print "End pose:"
print p
mg.set_pose_target(mg.get_random_pose())
traj = mg.plan()
print traj
