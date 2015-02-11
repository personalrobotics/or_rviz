#!/usr/bin/env python
import openravepy
import IPython

env = openravepy.Environment()
env.Load('data/wamtest1.env.xml')
env.SetViewer('RViz')

IPython.embed()
