from socketio.namespace import BaseNamespace

from bridge import get_planner

import logging
logger = logging.getLogger('moveit.socket')

class PlanNamespace(BaseNamespace):
    name = '/plan'

    planner = None
    def on_connected(self, *args):
        logger.debug('user %s connected' % self.request.user)
        planner = get_planner()
        planner.set_socket(self)
        self.emit('status','ready to go')

