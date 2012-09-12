#!/usr/bin/env python
# very elementary script to test the plugin loading

from openravepy import *

RaveInitialize()
RaveLoadPlugin('install/openprm')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    prmplan = RaveCreateProblem(env,'prmproblem')
    print prmplan.SendCommand('TestPrmGraph')
    raw_input('Done with testing')
finally:
    RaveDestroy()
