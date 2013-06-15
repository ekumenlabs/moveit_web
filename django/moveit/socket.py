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
        self.emit('status',{'text':'ready to plan','ready':True})
        self.ready = True

    def on_plan_random(self, *args):
        self.planner.plan_to_random_goal()

    def on_get_link_poses(self):
        if self.ready:
            self.emit('link_poses', self.planner.get_link_poses())

    def on_deleteme_test(self):
        self.planner.deleteme_joint_update()

    def on_scene_changed(self, scene):
        self.planner.set_scene(scene)
