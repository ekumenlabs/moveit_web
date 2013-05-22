# Create your views here.
from django.http import HttpResponse
import json

def run(request):
    data = json.dumps({})
    return HttpResponse(data, mimetype='application/json')

def goals(request):
    data = json.dumps({})
    return HttpResponse(data, mimetype='application/json')
