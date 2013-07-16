from socketio.namespace import BaseNamespace

from bridge import get_planner

import logging
logger = logging.getLogger('moveit.socket')

class PlanNamespace(BaseNamespace):
    name = '/plan'

    planner = None
    ready = False
    def on_connected(self, *args):
        logger.debug('user %s connected' % self.request.user)
        self.planner = get_planner()
        self.planner.set_socket(self)
        self.emit('status', self.planner.status)
        if self.planner.current_scene is not None:
            self.emit('current scene', self.planner.current_scene)
        self.ready = True

    def on_goal_random(self, *args):
        self.planner.set_random_goal()

    def on_plan_to_poses(self, poses):
        self.planner.plan_to_poses(poses)

    def on_get_link_poses(self):
        if self.ready:
            self.emit('link_poses', self.planner.get_link_poses())

    def on_deleteme_test(self):
        self.planner.deleteme_joint_update()

    def on_scene_changed(self, scene):
        self.planner.set_scene(scene)
