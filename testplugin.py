#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/openprm')
try:
    env=Environment()
    env.Load('scenes/opscene.env.xml')
    openprm = RaveCreatePlanner(env,'openprm')
    print openprm.SendCommand('help')
finally:
    RaveDestroy()
