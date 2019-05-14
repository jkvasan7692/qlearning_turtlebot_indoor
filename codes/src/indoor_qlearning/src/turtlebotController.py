#!/usr/bin/env python3

import numpy as np
import rospy
from gazeboEnvironment import GazeboEnvironment
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

import math
#import tf

#%%

class TurtlebotController(GazeboEnvironment):

    def __init__(self, gridSize):
        "Initialization function of the turtlebot controller. "
        rospy.logdebug("Start TurtleBotEnv INIT...")

        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboEnvironment(False,"WORLD")
        rospy.logdebug("COMPLETED init RobotGazeboEnv")

        self.gazebo.unpauseSim()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback)

        self._cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.set_model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=100)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        self._current_position = np.array([0 , 0])
        self._gridSize = gridSize

        rospy.logdebug("Finished TurtleBot2Env INIT...")

    def _check_all_sensors_ready(self):
        "Check if all sensors of the system is initialized"
        rospy.logdebug("START ALL SENSORS READY")
        self._check_model_states_ready()
        self._check_odom_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        "Check if the odom data is ready"
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
          try:
            self.odom = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
            rospy.logdebug("Current /odom READY=>")
          except:
            rospy.logerr("Current /odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_laser_scan_ready(self):
        "Check if the laser scan data is ready"
        self.laser_scan = None
        rospy.logdebug("Waiting for /kobuki/laser/scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
          try:
            self.laser_scan = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)
            rospy.logdebug("Current /kobuki/laser/scan READY=>")
          except:
            rospy.logerr("Current /kobuki/laser/scan not ready yet, retrying for getting laser_scan")

        return self.laser_scan

    def _check_model_states_ready(self):
        "Check if the gazebo model states are ready"
        self.model_states = None
        rospy.logdebug("Waiting for /gazebo/model_states to be READY...")
        while self.model_states is None and not rospy.is_shutdown():
          try:
            self.model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
            rospy.logdebug("Current /gazebo/model_states READY=>")
          except:
            rospy.logerr("Current /gazebo/model_states not ready yet, retrying for getting model_states")

        return self.model_states

    def _odom_callback(self, data):
        "Callback to update the odom data"
        self.odom = data

    def _laser_scan_callback(self, data):
        "Callback to update the laser scan ros data"
        self.laser_scan = data

    def _model_states_callback(self, data):
        "Callback to update the gazebo model states"
        self.model_states = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
          rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
          try:
            rate.sleep()
          except rospy.ROSInterruptException:
            # This is to avoid error when world is rested, time when backwards.
            pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")
        rospy.logdebug("All Publishers READY")


    def move_base(self, linear_speed, angular_speed):
        """It will move the base based at the given linear and angular speeds at 10 Hz.
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("TurtleBot2 Base Twist Cmd>>{}".format(cmd_vel_value))
        self._check_publishers_connection()
        rate = rospy.Rate(100)
        for _ in range(10):
          self._cmd_vel_pub.publish(cmd_vel_value)
          rospy.logdebug("Moving base with cmd_vel: {}".format(cmd_vel_value))
          rate.sleep()

    def get_laser_scan(self):
          return self.laser_scan

    def get_model_states(self):
          return self.model_states

    def _set_init(self, initPosition):
        """
        Set initial condition for simulation
        1. Set turtlebot at a random pose inside crib by publishing /gazebo/set_model_state topic
        2. Set a goal point inside crib for turtlebot to navigate towards

        Returns:
          init_position: array([x, y])
          goal_position: array([x, y])

        """
        rospy.logdebug("Start initializing robot...")
        # Set turtlebot inside crib, away from crib edges
        x = initPosition[0]
        y = initPosition[1]
        self.init_position = np.array([x, y])
        self.previous_position = self.init_position
        self._current_position = self.init_position
        w = 0
        model_state = ModelState()
        model_state.model_name = "mobile_base"
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 1
        model_state.pose.orientation.w = w
        model_state.reference_frame = "world"
        rate = rospy.Rate(100)
        for _ in range(10):
          self.set_model_state_publisher.publish(model_state)
          rate.sleep()

        rospy.logdebug("Robot was initiated as {}".format(model_state.pose))

    def _reset_sim(self):
        """Resets a simulation
        """
        rospy.logdebug("START robot gazebo _reset_sim")
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()
        self._check_all_sensors_ready()
        self.gazebo.pauseSim()
        rospy.logdebug("END robot gazebo _reset_sim")

        return True

    def _observe(self):
        """
        Here we define states as gazebo model_states
        :return:
        7 observations
        [
          x,
          y,
          v_x,
          v_y,
          cos(yaw),
          sin(yaw),
          yaw_dot
        ]
        """
#        print("Start Get Observation ==>")
        # We get the model states data
        model_states = self.get_model_states()
        rospy.logdebug("Turtlebot is @ state of {}".format(model_states))
        x = model_states.pose[-1].position.x # turtlebot was the last model in model_states
        y = model_states.pose[-1].position.y
#        self._current_position = np.array([x, y])
        v_x = model_states.twist[-1].linear.x
        v_y = model_states.twist[-1].linear.y
        quat = np.array([model_states.pose[-1].orientation.x, model_states.pose[-1].orientation.y,model_states.pose[-1].orientation.z, model_states.pose[-1].orientation.w])

        quaternion = R.from_quat(quat)

        euler = quaternion.as_euler(seq='xyz', degrees=True)
#        print("Euler Angles: ", euler)
#        euler = tf.transformations.euler_from_quaternion(quat)
#        cos_yaw = math.cos(euler[2])
#        sin_yaw = math.sin(euler[2])
#        yaw_dot = model_states.twist[-1].angular.z

        observations = np.array([x, y, v_x, v_y, euler[2]])
#        print("Observations ==> {}".format(observations))
        return observations

    def resetPosition(self , initPosition):
        "Reset the position of the robot to a random position in the environment"
        print("Reseting RobotGazeboEnvironment")
        self._reset_sim()
        self.gazebo.unpauseSim()
        self._set_init(initPosition)
        self.gazebo.pauseSim()
        obs = self._observe()
        print("END Reseting RobotGazeboEnvironment")
        self._current_position = initPosition.copy()
        self.orientationPosition = obs[4]
        print("Observation: ",obs)
        return

    def rotateBotInDirection(self , actionCmd):
        "Rotate bot in direction of the heading for the turtlebot"
        if(actionCmd[0] == 1):
            eulerRotation = 0
        elif(actionCmd[0] == -1):
            eulerRotation = 180
        elif(actionCmd[1] == 1):
            eulerRotation = 90
        else:
            eulerRotation = -90

        rotationAngle = eulerRotation - self.orientationPosition
        if(rotationAngle > 180):
            rotationAngle -= 360
        elif(rotationAngle < -180):
            rotationAngle -= -360

        print("Euler : ",eulerRotation, "Current Orientation : " , self.orientationPosition, "Rotation Angle : ", rotationAngle)
        return eulerRotation , rotationAngle

    def moveRobot(self , actionCmd):
        "Move robot based on an action. Consider a 4 connected action space. In this function the robot is first rotated in the direction of the heading. It then moves to the new position"
        obstacleHit = False
        positionInc = actionCmd*self._gridSize

        obs = self._observe()
        self._current_position = obs[0:2]
        
        scaledPos = self._current_position.copy() - np.array([-5 , -5])
        
        normalizedPos  = scaledPos - scaledPos % self._gridSize
        posToStart = normalizedPos + np.array([-5 , -5])
        
        newPosition = posToStart + positionInc.copy() + np.fabs(positionInc.copy())*0.5
                
        print("Turtlebot Current Position: ", self._current_position, "Position To Start: ", posToStart,  "New Position: ", newPosition, "Scaled: ", scaledPos, "normalizedPos: ", normalizedPos )
        eulerAngle , rotationAngle = self.rotateBotInDirection(actionCmd)
        cmd_vel_value = Twist()
        angleStep = 3.14/180 * 30 * rotationAngle/math.fabs(rotationAngle)
        self.gazebo.unpauseSim()
        rate = rospy.Rate(100)
        prevRotation = self.orientationPosition.copy()
        # conver to 0 to 360
        if(prevRotation < 0):
            prevRotation += 360
        rotationInc = 0
        while(math.fabs(rotationInc) < (math.fabs(rotationAngle))):
            cmd_vel_value.linear.x = 0
            cmd_vel_value.angular.z = angleStep
            self._cmd_vel_pub.publish(cmd_vel_value)
            rate.sleep()
            obs = self._observe()
            self.orientationPosition = obs[4]
            newAngle = self.orientationPosition.copy()
            if(newAngle < 0):
                newAngle += 360
            
            angleDiff = newAngle - prevRotation.copy()
            if(math.fabs(angleDiff) > 100 ):
                # This means that the rotation is from 4th quadrant to 1st quadrant or vice versa
                estimateRotationInc = (math.fabs(angleDiff) - 360)*angleDiff/math.fabs(angleDiff)
                angleDiff = estimateRotationInc
            rotationInc += angleDiff
            print("Rotation Increment: ",rotationInc, "Prev Rotation: ", prevRotation, "Current Orientation: ", self.orientationPosition, "angle Increment: ", angleDiff)
            prevRotation = newAngle.copy()
            
        # The current oritnetaion i sin prevRotation we make use of this
        # Check if there is an obstacle ahead of the robot
#        print("Laser_Scan Data:",self.laser_scan)
            
        obs = self._observe()
        
        self._current_position = obs[0:2]
        self.orientationPosition = obs[4]

        count = len(self.laser_scan.ranges)
        
        if(80 < math.fabs(self.orientationPosition) < 100):
            rangeCheck = math.fabs(newPosition[1] - self._current_position[1])
        else:
            rangeCheck = math.fabs(newPosition[0] - self._current_position[0])
        
        print("Checking Laser range: ", rangeCheck)
        for ind in range(int(count/2 - 10) , int(count/2 + 10)):
            if(self.laser_scan.ranges[ind] < rangeCheck+0.5):
                print("Obstacle Hit is True")
                obstacleHit = True

        if(False == obstacleHit):
            print("MoveHead")
            prevPosition = self._current_position.copy()
            
            goalPosition = newPosition.copy()*actionCmd.copy()
            beginPosition = self._current_position.copy()*actionCmd.copy()
            distToReach = np.linalg.norm(goalPosition - beginPosition)
            robotPosInc = 0
            while(math.fabs(robotPosInc) <= math.fabs(distToReach)):
                cmd_vel_value.linear.x = 0.25
                cmd_vel_value.angular.z = 0
                self._cmd_vel_pub.publish(cmd_vel_value)
                rate.sleep()
                obs = self._observe()
                self._current_position = obs[0:2]
                currentPosDir = self._current_position.copy()*actionCmd.copy()
                print("Prev Position: ", prevPosition , "Start Position: ", posToStart,  " Current Position: ", self._current_position)
                robotPosInc = np.linalg.norm(currentPosDir.copy() - beginPosition.copy())
                print("Position Increment: ", robotPosInc)

#            self._current_position = newPosition.copy()
            print("Updated position: ", self._current_position)

        self.gazebo.pauseSim()

        return obstacleHit , self._current_position.copy()
