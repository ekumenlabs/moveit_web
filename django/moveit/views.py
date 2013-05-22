# Create your views here.
from django.http import HttpResponse
import json
from bridge import get_planner

def run(request):
    get_planner().calculate_goals()
    data = json.dumps({'rc':'ok'})
    return HttpResponse(data, mimetype='application/json')

def goals(request):
    data = json.dumps(get_planner().get_goals_as_json())
    return HttpResponse(data, mimetype='application/json')
