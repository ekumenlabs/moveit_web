from moveit_ros_planning_interface import _moveit_move_group_interface
import rospy
import tf

rospy.init_node('plan_base')
tf_broadcaster = tf.TransformBroadcaster()
rospy.sleep(1)

def print_base_link(links):
    print "Base coords: %s" % links['global_link'][0]

moveit = _moveit_move_group_interface.MoveGroup("base")
print "Starting"

print "Initial links:"
print_base_link(moveit.get_link_poses_compressed())

# This will compute a plan for [x,y,theta] on the floor space
moveit.set_joint_value_target([1.5,0.0,3.1415])
plan = moveit.compute_plan()
# print "Plan: %s" % plan
trans = plan['multi_dof_joint_trajectory']['points'][-1]['transforms'][0]
print "Transform: %s" % trans

# Now:
# 1- how do I get the updated poses given this plan? It's a multidof trajectory with frames
# as steps, not joint values
# 2- How do I update the initial pose with the end state in the robot_state, so that I ca
# plan from this new location?

# Update and get location?
print "Updated links"
print_base_link(moveit.update_robot_state_base("base_footprint", trans, True))

# Try updating the base position by publishing TF
tf_broadcaster.sendTransform((1.5,0.0,0.0),
        tf.transformations.quaternion_from_euler(0,0,3.1415),
        rospy.Time.now(),
        "base_footprint", "odom_combined")
rospy.sleep(3)

# Plan again with (supposedly) new start
moveit.set_joint_value_target([1.5,1.5,0.0])
plan = moveit.compute_plan()

print "Final links"
trans = plan['multi_dof_joint_trajectory']['points'][-1]['transforms'][0]
print_base_link(moveit.update_robot_state_base("base_footprint", trans, True))
