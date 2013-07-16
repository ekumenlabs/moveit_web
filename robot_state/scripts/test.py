#!/usr/bin/env python

from moveit_web_robot_state import moveit_web_robot_state
mvt = moveit_web_robot_state.WebRobotStateWrapper()
print mvt.getLinkPoses()
