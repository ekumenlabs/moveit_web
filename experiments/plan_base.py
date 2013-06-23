from moveit_ros_planning_interface import _moveit_move_group_interface

moveit = _moveit_move_group_interface.MoveGroup("base")
print "Starting"

print "Current links: %s" % moveit.get_link_poses_compressed()

# This will compute a plan for [x,y,theta] on the floor space
moveit.set_joint_value_target([1.5,0.0,3.1415])
plan = moveit.compute_plan()
print "Plan: %s" % plan
trans = plan['multi_dof_joint_trajectory']['points'][-1]['transforms'][0]
print "Transform: %s" % trans

# Now:
# 1- how do I get the updated poses given this plan? It's a multidof trajectory with frames
# as steps, not joint values
# 2- How do I update the initial pose with the end state in the robot_state, so that I ca
# plan from this new location?

# Update and get location?
print "Updated links: %s" % moveit.update_robot_state_base("base_footprint", trans, True)

# Plan again with (supposedly) new start
moveit.set_joint_value_target([1.5,1.5,0.0])
plan = moveit.compute_plan()
