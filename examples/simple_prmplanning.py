from __future__ import with_statement
__author__ = 'Billy Okal'
__copyright__ = 'CopyRight(C) Billy Okal 2010-2011'
__license__ = 'BSD'

from openravepy import *
import time
from numpy import *
from optparse import OptionParser

class PRMPlanner:
	"""PRMPlanner planning Demonstration"""
	def __init__(self, env, planner):
		self.env = env
		self.env.SetDebugLevel(DebugLevel.Debug)
		self.planner = planner
		self.robot = self.env.GetRobots()[0]
		self.problem = RaveCreateProblem(self.env, "prmplanning")
		args = self.robot.GetName()
		args += ' planner '
		args += self.planner
		self.env.LoadProblem(self.problem, args)
		T=array([0,0,0,0,0,0,0,0,0,0,0])	# set an initial position
		self.robot.SetActiveDOFValues(T)
		
	def Serialize(self, T):
		return 'goal %s'%(' '.join(str(f) for f in T))
		
	def runPlanner(self, goal):
		cmd = 'RunPRM goal ' + ' '.join(str(f) for f in goal) + ' '
		
		#if outputtraj is not None:
		#	cmd += 'writetraj %s'%outputtraj
		
		res = self.problem.SendCommand(cmd)
		
		#res = self.problem.SendCommand('RunQuery')
		
		print 'Command Sent',cmd
		
		if res is None:
			print 'planning error with %s'%self.planner
		
		return res
		
def run():
	parser = OptionParser(description='Motion Plannig using PRMs.')
	parser.add_option('--planner', action="store", type='string', dest='planner', default='sblplanner', help='PRM planner to use (default=%default) \nChoices are: \nsblplanner \n\tvprmplanner \n\tclassicprm')
	parser.add_option('--scene', action="store", type='string', dest='scene', default='data/lab1.env.xml', help='Scene file to load')
	(options, args) = parser.parse_args()
	
	env = Environment()
	try:
		env.SetViewer('qtcoin')
		env.Load(options.scene)
		time.sleep(0.1) 
		prm = PRMPlanner(env, options.planner)
		T1 = [-0.85, 1.34, -0.074, 2.23, -1.26, -1.558, 1.59]
		T2 = [2.45, 1.24, -0.064, 2.13, -5.16, -1.648, 1.19]
		T3 = [-0.85, 1.34, -0.074, 2.23, -1.26, -1.558, 1.59]
		
		raw_input('Press any key to start')
		res = prm.runPlanner(goal=[-0.85, 1.34, -0.074, 2.23, -1.26, -1.558, 1.59])#, outputtraj='trajoutput.txt')
		time.sleep(2)
        
        
	finally:
		raw_input('Press any key to quit')
		env.Destroy()

	
	
if __name__ == "__main__":
	run()
		
