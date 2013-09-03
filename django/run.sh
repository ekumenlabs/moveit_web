#!/bin/bash

# Ensure the directory for uploads exists.
[ -d '/home/vagrant/moveit/src/moveit_web/django/media' ] || mkdir -p '/home/vagrant/moveit/src/moveit_web/django/media'

# Source the ROS environment.
. ../../../devel/setup.bash

# Launch the webserver.
./runserver.py
