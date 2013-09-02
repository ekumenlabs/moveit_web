from socketio.namespace import BaseNamespace
from django.conf import settings
from os.path import join

from bridge import get_planner
from models import Scene

import logging
logger = logging.getLogger('moveit.socket')

MESHES_ROOT = join(settings.PROJECT_ROOT, 'static', 'meshes')


class PlanNamespace(BaseNamespace):
    name = '/plan'

    planner = None
    ready = False

    def on_connected(self, *args):
        self.planner = get_planner()
        self.planner.set_socket(self)

        self.emit('status', self.planner.status)
        self.ready = True

        self.emit('link_poses', self.planner.get_link_poses())
        self._emit_current_scene()

    def _emit_current_scene(self):
        if self.planner.get_scene() is None:
            blank_scene = Scene.objects.get(name="Blank")
            self.planner.set_scene(blank_scene, MESHES_ROOT)
        self.emit('current scene', self.planner.get_scene().to_dict())

    def on_goal_random(self, *args):
        self.planner.set_random_goal()

    def on_plan_to_poses(self, poses):
        self.planner.plan_to_poses(poses)

    def on_get_link_poses(self):
        if self.ready:
            self.emit('link_poses', self.planner.get_link_poses())

    def on_change_scene(self, scene_name):
        # This method does two things:
        # * set the scene for the MoveIt! layer
        # * send the scene data to the client
        # Although both operations can be performed in parallel, they are here
        # done sequentially in the interest of KISS
        try:
            scene = Scene.objects.get(name=scene_name)
            self.planner.set_scene(scene, MESHES_ROOT)
            self.emit('changed scene', scene)
        except Scene.DoesNotExist:
            logger.error('Client tried to change to an inexistent scene.')
