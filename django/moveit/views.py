# Create your views here.
from django.http import HttpResponse, HttpResponseRedirect
from django.core.urlresolvers import reverse
from django.conf import settings
import os.path
import json


def run(request):
    #get_planner().calculate_goals()
    data = json.dumps({'rc': 'ok'})
    return HttpResponse(data, mimetype='application/json')


def goals(request):
    #data = json.dumps(get_planner().get_goals_as_json())
    data = {}
    return HttpResponse(data, mimetype='application/json')


def new_scene(request):
    "Save the bundle in 'payload' to disk."

    # New scenes may only be POSTed
    if request.method != "POST":
        return HttpResponse(status=405)

    payload = request.FILES.get('payload', None)
    if not payload:
        # return HttpResponse(status=400)
        return HttpResponseRedirect(reverse('home'))

    # Allow only data, for zip and tar files
    if hasattr(settings, 'ACCEPTED_CONTENT_TYPE'):
        accepted = settings.ACCEPTED_CONTENT_TYPE
        if payload.content_type not in accepted:
            return HttpResponse(status=415)

    # Refuse to propess files too big
    if hasattr(settings, 'MAX_UPLOAD_FILE_SIZE'):
        max_size = settings.MAX_UPLOAD_FILE_SIZE
        if payload.size > max_size:
            msg = 'Uploaded file is too big. Stick to %s' % max_size
            return HttpResponse(msg, status=413)

    # Overwrite any file previously written with the same name
    filename = os.path.join(settings.MEDIA_ROOT, payload.name)
    with open(filename, 'wb') as disk:
        for chunk in payload.chunks():
            disk.write(chunk)

    return HttpResponseRedirect(reverse('home'))
