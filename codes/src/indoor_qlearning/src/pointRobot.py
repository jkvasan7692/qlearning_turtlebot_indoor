import numpy as np
#%% Point Robot Simulator class.is system describes a point robot with 4 connected action space. 
class PointRobot:
    def __init__(self, gridSize, gridMap):
        "Initialization of the function is performed here"
        self._gridSize = gridSize
        self._currentPosition = np.array([0 , 0])
        self._gridMap = gridMap
        
    def reset(self, initPosition):
        "Resetting the position of the robot to a random position"
        self._currentPosition= initPosition
        
    def moveRobot(self , actionCmd):
        "Move robot based on an action. Consider a 4 connected action space"
        obstacleHit = False
        positionInc = actionCmd*self._gridSize
        
        newPosition = self._currentPosition + positionInc
        print("Current Position of the robot: ", self._currentPosition, "action command: ", actionCmd)
        
        gridLocation = newPosition / self._gridSize
        
        maxRows = self._gridMap.shape[0]
        maxCols = self._gridMap.shape[1]
        
        if(gridLocation[0] >= maxRows or gridLocation[1] >= maxCols or gridLocation[0] < 0 or gridLocation[1] < 0):
            obstacleHit = True
        elif( 1 == self._gridMap[int(gridLocation[0]), int(gridLocation[1])] ):
            # Obstacle Detected
            obstacleHit = True
        else:
            self._currentPosition = newPosition
        
        print("New Position of the robot: ", self._currentPosition, "Obstacle Hit flag: ", obstacleHit)
            
        return obstacleHit , self._currentPosition
        