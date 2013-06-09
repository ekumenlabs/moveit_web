from socketio.namespace import BaseNamespace

from bridge import get_planner

import logging
logger = logging.getLogger('moveit.socket')

class PlanNamespace(BaseNamespace):
    name = '/plan'

    planner = None
    def on_connected(self, *args):
        logger.debug('user %s connected' % self.request.user)
        self.planner = get_planner()
        self.planner.set_socket(self)
        self.emit('status',{'text':'ready to go'})

    def on_plan_random(self, *args):
        self.planner.plan_to_random_goal()
