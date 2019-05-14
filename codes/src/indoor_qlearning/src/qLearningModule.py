import numpy as np
import random
from common import *

#%% Q learning module of the system. This si the module that learns through the random movement and converges to an optimal policy
class QLearningSystem:
    def __init__(self, learnRate, discRate, epsilon, gridSize, qTable, numRows , numCols):
        "Initialization of the Q learning system class"
        self._learnRate = learnRate
        self._currentState = np.array([0 , 0])
        self._goalState = np.array([0, 0])
        self._discRate = discRate
        self._rewardData = {("Goal", 1000), ("Step", -1), ("Obstacle",-5)}

        self._prevState = np.array([0 , 0])
        self._actionTaken = np.array([0 , 0])
        self._gridSize = gridSize
        self._epsilon = epsilon

        self._numRows = numRows
        self._numCols = numCols
        self._qTable = qTable

        self._mapEndPos = np.array([10 , 10])
        self._mapStartPos = np.array([0 , 0])

        self._mode = "QTable"
        self._numActions = 4

    def mapInitEndPosition(self , mapStartPos , mapEndPos):
        "Init and end posiotn of the map is set here. These parameters are maintained in the class to calculate the states of the system"
        self._mapStartPos = mapStartPos.copy()
        self._mapEndPos = mapEndPos.copy()

    def decodeStateFromPosition(self , position):
        "Decoding the states of the Q table from the actual position of the robot"
        state = np.array([0 , 0])
        position -= self._mapStartPos.copy()
        state = np.floor(position / self._gridSize)
        print_debug("State:", state)

        return state

    def reset(self, startPos, goalPos):
        "Resetting the start and goal positon of the q learning module"
        startState = self.decodeStateFromPosition(startPos)
        goalState = self.decodeStateFromPosition(goalPos)
        print_debug("Start State and Goal State: ",startState , goalState)
        self._currentState = startState.copy()
        self._goalState = goalState.copy()

    def encodeActionSpace(self, actionInd):
        "Determine the action space for the action index. "
        if(self._numActions == 4):
            if(actionInd == 2):
                actionCmd = np.array([0 , 1])  # Right
            elif(actionInd == 1):
                actionCmd = np.array([0 , -1]) # Left
            elif(actionInd == 0):
                actionCmd = np.array([-1, 0])  # up
            else:
                actionCmd = np.array([1 , 0])  # Top
        else:
            if(actionInd == 0):
                actionCmd = np.array([-1,-1])
            elif(actionInd == 1):
                actionCmd = np.array([-1, 0])
            elif(actionInd == 2):
                actionCmd = np.array([-1, 1])
            elif(actionInd == 3):
                actionCmd = np.array([0, -1])
            elif(actionInd == 4):
                actionCmd = np.array([0, 1])
            elif(actionInd == 5):
                actionCmd = np.array([1, -1])
            elif(actionInd == 6):
                actionCmd = np.array([1, 0])
            elif(actionInd == 7):
                actionCmd = np.array([1, 1])
        print_debug("Action Command: ", actionCmd)

        return actionCmd


    def predict(self):
        "Function to predict the action based on epsilon greedy policy"

        if(self._mode == "QTable"):
            print_debug("Current State: ", self._currentState)
            qRowNum = int(self._currentState[0]*self._numCols + self._currentState[1])
            print_debug("Q Row Num: ",qRowNum)

            choice = random.uniform(0,1)
            print_debug("Random Seed: ", choice)

            if(choice < self._epsilon):
                actionInd = np.random.randint(self._numActions)
            else:
                maxQVal = np.max(self._qTable[qRowNum, :])
                actionPoss = np.where(self._qTable[qRowNum, :] == maxQVal)
                print_debug("Possible Actions: ", actionPoss)
                actionInd = np.random.choice(actionPoss[0])

            print_debug("Chosen Action: ", actionInd)
            actionCmd = self.encodeActionSpace(actionInd)

            self._prevState = self._currentState

            self._actionTaken = actionInd

        return actionCmd, actionInd

    def predictTest(self):

        if(self._mode == "QTable"):

            print_debug("Current State: ", self._currentState)
            qRowNum = int(self._currentState[0]*self._numCols + self._currentState[1])
            print_debug("Q Row Num: ",qRowNum)

            maxQVal = np.max(self._qTable[qRowNum, :])
            actionPoss = np.where(self._qTable[qRowNum, :] == maxQVal)
            print_debug("Possible Actions: ", actionPoss)
            actionInd = np.random.choice(actionPoss[0])

            print_debug("Chosen Action: ", actionInd)
            actionCmd = self.encodeActionSpace(actionInd)

        return actionCmd, actionInd

    def update(self , status,  turtlePosition):
        "Function to update the Q values based on the turtle bot position"
        newState = self.decodeStateFromPosition(turtlePosition)
        print_debug("New Position of bot: ", newState, "old Position: ", self._currentState)
        print_debug("New State: ", newState)
        diagonalAction = False
        if(np.fabs(np.linalg.norm(newState - self._currentState)) > 1):
            diagonalAction = True
        if(self._mode == "QTable"):
           if(1 == status):
                reward = -5
           elif(np.array_equal(self._goalState , newState)):
                reward = 1000
           else:
               reward = -1

           self._currentState = newState

           qRowNum = int(self._currentState[0]*self._numCols + self._currentState[1])
           print_debug("Q Row Number: ", qRowNum)

           if(1 == status):
               maxQVal = 0
           else:
               maxQVal = np.max(self._qTable[qRowNum , :])

           qPrevRowNum = int(self._prevState[0]*self._numCols + self._prevState[1])
           print_debug("Q Row Num: ", qPrevRowNum)

           self._qTable[qPrevRowNum , self._actionTaken] = self._qTable[qPrevRowNum , self._actionTaken] + self._learnRate*(reward + self._discRate*maxQVal - self._qTable[qPrevRowNum , self._actionTaken])

        return
           
    def updateCurrentState(self, turtlePosition):
       "Updating the current state of the robot based on the turtlebot position. Unlike the previous function, this is used only to update the state of the system"
       newState = self.decodeStateFromPosition(turtlePosition)
       self._currentState = newState
       return

    def goalReached(self):
        "Function checks if the current state of the system is equal to the goal state"
        print_debug("Current State and Goal State: ", self._currentState, self._goalState)
        if(np.array_equal(self._currentState,self._goalState)):
             return True
        else:
             return False
