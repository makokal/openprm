#!/usr/bin/env python
# very elementary script to test the plugin loading

from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/openprm')
try:
    env=Environment()
    env.Load('scenes/opscene.env.xml')
    prmplan = RaveCreateProblem(env,'prmplanning')
    print prmplan.SendCommand('TestPrmGraph')
    raw_input('Done with testing')
finally:
    RaveDestroy()
