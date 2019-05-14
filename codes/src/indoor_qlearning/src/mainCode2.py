#! /usr/bin/env python3
#%%
import numpy as np

import common
from pointRobot import *
from qLearningModule import *
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from turtlebotController import *

import time

#%% - Class Initialization  - Initialize 3x3 grid for a startup
MapStartLocation = np.array([-5 , -5])
MapEndLocation = np.array([5,5])
numActions = 4
gridSize = 4

numRows = int((MapEndLocation[0] - MapStartLocation[0]) / gridSize)+1
numCols = int((MapEndLocation[1] - MapStartLocation[1]) / gridSize)+1


QTable = np.zeros((numRows*numCols, 4))

numEpisodes = 100000

#%% Q Learning Parameters:
learnRate = 0.5
discRate = 0.5
epsilon = 0.2

maxRange = 3

#%% - Initializing the parameters. The class instantiations is performed here wherein the turtlebot controller object is created here.
rospy.init_node('qLearner')
turtlebot = TurtlebotController(gridSize)
qLearnSystem = QLearningSystem(learnRate , discRate, epsilon, gridSize, QTable, numRows, numCols)
qLearnSystem.mapInitEndPosition(MapStartLocation.copy() , MapEndLocation.copy())

goalLocation = np.array([5 - 0.5, 5 - 0.5])

#%% Trial tests here


#%% Training of the system. The system navigates in its environment and learns the optimal policy
for ind in range(numEpisodes):
    randomStart = np.random.randint(low =0 , high= maxRange , size=2)
    botStartLocation = randomStart - randomStart % gridSize + gridSize/2
    turtlebot.resetPosition(botStartLocation.copy() + MapStartLocation.copy())

    qLearnSystem.reset(randomStart.copy() , goalLocation.copy())
    prevQTable = np.zeros(QTable.shape)
    while(False == qLearnSystem.goalReached()):
        actionCmd, actionInd = qLearnSystem.predict()
        obstacleHit , returnedPosition = turtlebot.moveRobot(actionCmd.copy())
        print("New Position is: ", returnedPosition)
        qLearnSystem.update(obstacleHit , returnedPosition.copy())

        print_debug(QTable)
    if(True == np.allclose(QTable , prevQTable, rtol = 10**-1)):
        print("The two tables are equal")
    prevQTable = QTable
