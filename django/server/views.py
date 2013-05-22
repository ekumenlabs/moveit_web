import django.contrib.auth
from django.shortcuts import redirect, render
from django.http import HttpResponse
import logging

logger = logging.getLogger('server.views')

def login(request):
    return render(request,'login.html')

def login_post(request):
    logger.info("logging in user %s" % request.POST['username'])
    username = request.POST['username']
    password = request.POST['password']
    user = django.contrib.auth.authenticate(username=username, password=password)
    if user is not None:
        django.contrib.auth.login(request, user)
        return redirect('/')
    else:
        return HttpResponse('Login error, try again')
