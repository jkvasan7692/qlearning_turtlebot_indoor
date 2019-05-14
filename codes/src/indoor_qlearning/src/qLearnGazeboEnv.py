#!/usr/bin/env python3

import numpy as np
import rospy
from gazeboEnvironment import GazeboConnection

#%%

class qLearnGazeboEnv():
    def __init__(self, start_init_physics_parameters=True, reset_world_or_sim="WORLD"):
        # To reset Simulations
        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(start_init_physics_parameters,reset_world_or_sim)
        rospy.logdebug("END init RobotGazeboEnv")