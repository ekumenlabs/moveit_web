#!/usr/bin/env python

# Patch everything to work with gevent (i.e.: convert blocking calls in
# cooperative co-routines)
from gevent import monkey; monkey.patch_all()
from socketio.server import SocketIOServer

# TODO: Make this a parameter
DEFAULT_PORT = 8080

# The following two lines are enough to get access to django
# methods from within this code
import os
os.environ['DJANGO_SETTINGS_MODULE'] = 'server.settings'

import sys
import traceback

from django.core.wsgi import get_wsgi_application
from django.core.signals import got_request_exception

# This is necessary to print exceptions occurred in gevent-socketio
def exception_printer( sender, **kwArgs):
    # TODO: Parameterize and/or send to logs
    print >> sys.stderr, ''.join(traceback.format_exception(*sys.exc_info()))
got_request_exception.connect(exception_printer)

# Build WSG Application pipeline with django and
# the static files handler
application = get_wsgi_application()
from django.contrib.staticfiles.handlers import StaticFilesHandler
application = StaticFilesHandler(application)

# Start SocketIO server
print
print 'Django+SocketIO Listening on port %s' % DEFAULT_PORT
SocketIOServer(('', int(DEFAULT_PORT)), application,
   # Number of seconds between heartbeats server-client.
   #heartbeat_interval=5,

   # Number of seconds to wait for a heartbeat. If this
   # timeout is not met, the connection is considered lost.
   #heartbeat_timeout=15,

   resource='socket.io',
   policy_server=False).serve_forever()
