from moveit_ros_planning_interface import _moveit_move_group_interface

moveit = _moveit_move_group_interface.MoveGroup("right_arm_and_torso")
print "Root link: "
print moveit.get_robot_root_link()
print "Root state: "
print moveit.get_link_poses_compressed()
