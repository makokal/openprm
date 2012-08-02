#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/openprm')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    PRMPlanner = RaveCreatePlanner(env,'PRMPlanner')
    print PRMPlanner.SendCommand('help')
finally:
    RaveDestroy()
