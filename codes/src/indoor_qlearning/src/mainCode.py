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

numActions = 4
#%%
def updatePlot(robotPosition, actionInd, mode="train"):
    "This Function is used to update the plot of the Q table."
    global text_list , plt, QTable, all_axes, prevAxesInd, numCols, numActions
    # gridspec inside gridspec
    for ind in range(len(text_list)):
        stateInd = int(np.floor(ind/numActions))
        actionInd = ind % numActions
#        print(stateInd , actionInd)
        text_list[ind].set_text(str(np.around(QTable[stateInd , actionInd], decimals=3)))

    axesInd = int((robotPosition[0] * numCols + robotPosition[1])*9)


    if(mode == "train"):
        for ind in range(9):
            if(prevAxesInd>-1):
                all_axes[prevAxesInd+ind].set_axis_bgcolor('white')
            all_axes[axesInd+ind].set_axis_bgcolor('yellow')
        prevAxesInd = axesInd
    elif(mode == "test"):
        for ind in range(9):
            all_axes[axesInd+ind].set_axis_bgcolor('blue')

    plt.pause(0.001)

def getRandomStart(low, high, size):
    "Function to generate a random point to reset the robot in a position"
    randomStart = np.random.randint(low=low,high=high,size=size)
    while gridMap[randomStart[0]][randomStart[1]] == 1:
        randomStart = np.random.randint(low=low,high=high,size=size)
    return randomStart

def updateRandomObstacles(gridMap, numOfObstacles):
    "This function is used to update the obstacles in random positon. The function had been used to analyse the number of iterations vs obstacles"
    obstacleCount = 0
    goalRange = maxRange-1
    while obstacleCount < numOfObstacles:
        point = np.random.randint(low=0,high=maxRange,size=2)
        if (gridMap[point[0]][point[1]] != 1 and point[0] < goalRange and
                point[1] < goalRange):
            gridMap[point[0]][point[1]] = 1
            obstacleCount += 1
    print(gridMap)
    return gridMap
#%% - Class Initialization  - Initialize 3x3 grid for a startup

# The grid map array to simulate the environment. This is used in the point Robot simulator function
gridMap = np.array([[0 ,0, 0 , 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0 ,0, 0 , 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
#gridMap = np.array([[0 ,0, 0 , 0, 0], [0, 0, 1, 1, 0],[0, 0, 0, 1, 0], [0, 0, 1, 1, 0], [0, 0, 0, 0, 0]])
maxRange = gridMap.shape[0]

MapStartLocation = np.array([0 , 0])
MapEndLocation = np.array([maxRange,maxRange])
#numActions = 4
gridSize = 1
prevAxesInd = -1

numRows = int(MapEndLocation[0] / gridSize)
numCols = int(MapEndLocation[1] / gridSize)

# Q table generated here. 
QTable = np.zeros((numRows*numCols, numActions))

numEpisodes = 100000
saturationThreshold = 500

#%% Q Learning Parameters:
learnRate = 0.5
discRate = 0.5
epsilon = 0.2



#%% - Initializing the parameters
pointRobot = PointRobot(gridSize, gridMap)
qLearnSystem = QLearningSystem(learnRate , discRate, epsilon, gridSize, QTable, numRows, numCols)

goalLocation = np.array([maxRange - 0.5, maxRange - 0.5])

#%% Trial tests here
#%% Updating the plots
#%% Plotting the final Q table
fig = plt.figure(figsize=(8, 8))
plt.ion()

# gridspec inside gridspec
outer_grid = gridspec.GridSpec(numRows, numCols, wspace=0.0, hspace=0.0)
text_list = list()

for ind in range(numRows*numCols):
    obstacleDetected = False
    inner_grid = gridspec.GridSpecFromSubplotSpec(3, 3, subplot_spec=outer_grid[ind], wspace=0.0, hspace=0.0)

    gridColNum = ind % numCols
    gridRowNum = int(np.floor(ind / numCols))
    if(gridMap[gridRowNum , gridColNum] == 1):
        obstacleDetected = True

    for ind2 in range(0,9):
        ax = plt.Subplot(fig, inner_grid[ind2])
        if(True == obstacleDetected):
            ax.set_axis_bgcolor('red')
        if(numActions == 8):
            if(ind2 == 0):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 0], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 1):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 1], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 2):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 2], decimals = 3), fontsize=14)
                text_list.append(t1)

            elif(ind2 == 3):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 3], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 5):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 4], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 6):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind , 5], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 7):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 6], decimals = 3), fontsize=14)
                text_list.append(t1)
            elif(ind2 == 8):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 7], decimals = 3), fontsize=14)
                text_list.append(t1)

        else:
            if(ind2 == 1):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 0], decimals = 3), fontsize=12)
                text_list.append(t1)
            elif(ind2 == 3):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 1], decimals = 3), fontsize=12)
                text_list.append(t1)
            elif(ind2 == 5):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind, 2], decimals = 3), fontsize=12)
                text_list.append(t1)
            elif(ind2 == 7):
                t1 = ax.text(0.5, 0.5, np.around(QTable[ind , 3], decimals = 3), fontsize=12)
                text_list.append(t1)
        ax.set_xticks([])
        ax.set_yticks([])
        fig.add_subplot(ax)


all_axes = fig.get_axes()

for ax in all_axes:
    for sp in ax.spines.values():
        sp.set_visible(False)
    if ax.is_first_row():
        ax.spines['top'].set_visible(True)
    if ax.is_last_row():
        ax.spines['bottom'].set_visible(True)
    if ax.is_first_col():
        ax.spines['left'].set_visible(True)
    if ax.is_last_col():
        ax.spines['right'].set_visible(True)

plt.draw()

plt.pause(5)
#%% Training face being performed here. The point robot explores its environment and learns the optimal policy
prevQTable = np.zeros(QTable.shape)
for ind in range(numEpisodes):
    randomStart = getRandomStart(0,maxRange,2)
    pointRobot.reset(randomStart)
    qLearnSystem.reset(randomStart , goalLocation)
    while(False == qLearnSystem.goalReached()):
        actionCmd, actionInd = qLearnSystem.predict()
        obstacleHit , newPosition = pointRobot.moveRobot(actionCmd)
        qLearnSystem.update(obstacleHit , newPosition)
        newState = qLearnSystem.decodeStateFromPosition(newPosition)

        print_debug(QTable)
#        updatePlot(newState , actionInd)
    if np.allclose(QTable , prevQTable, atol=1e-11, equal_nan=True):
        saturation += 1
        if saturation > saturationThreshold:
            print("----> QTable Saturated >>> Start location: ",randomStart, "Goal Location: ", goalLocation)
#            results.append([learningRate, (ind-saturation+1), np.linalg.norm(QTable.copy()), numOfObstacles])
#            qtables.append(QTable)
            break
    else:
        saturation = 0
    prevQTable = QTable

#%% Testing for a random position
randomStart = np.array([0.5,0.5])

pointRobot.reset(randomStart.copy())
qLearnSystem.reset(randomStart.copy() , goalLocation.copy() )
while(False == qLearnSystem.goalReached()):
    actionCmd, actionInd = qLearnSystem.predictTest()
    obstacleHit , newPosition = pointRobot.moveRobot(actionCmd)
    newState = qLearnSystem.decodeStateFromPosition(newPosition)
    qLearnSystem.updateCurrentState(newPosition)
    updatePlot(newState , actionInd, mode="test")

print_debug(QTable)

plt.pause(200)
