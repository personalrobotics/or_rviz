#!/usr/bin/env python
import openravepy

robot_uri = '/opt/pr/herb_description/ordata/robots/herb.robot.xml'
#robot_uri = 'robots/barrettwam.robot.xml'
#robot_uri = '/home/mkoval/backup/ros/r2py/ordata/robots/r2.robot.xml'

openravepy.misc.InitOpenRAVELogging()

env = openravepy.Environment()
body = env.ReadRobotURI(robot_uri)
body.SetName('robot')
env.Add(body)

openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)
env.SetViewer('InteractiveMarker openrave_interactivemarker')
