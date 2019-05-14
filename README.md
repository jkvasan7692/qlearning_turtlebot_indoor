# Autonomous Navigation using Reinforcement Learning for Indoor Environment

This project describes autonmous navigation using q learning for indoor environment divided into 10x10 grid. Further the simulation is performed on turtlebot in gazebo ROS

## Getting Started

### Prerequisites

The code is implemented in Python and has the following Dependency :
1. Python 3
2. Gazebo 6
3. ROS Kinetic distribution
4. Turtlebot Gazebo kinetic package

### Installing

Create a folder in which you would like to clone the project repository.

Execute the following command:
git clone https://github.com/jkvasan7692/qlearning_turtlebot_indoor.git

### Compilation

Go to the codes folder and execute the following command: \\
catkin build

### Directory and Files
The project is organized as ROS package that can be downloaded, built and executed using ROS command
The project directory structure is as follows:
1. codes - Contains the source code Files
-- indoor_qlearning - ROS package
  -- src : The source files as explained below
  * pointRobot.py - Point Robot Simulator class. Simulates the point robot behavior with 4 connected action space.
  * turtlebotController.py - Controller Source code of the turtlebot controller module. Its functionality is to control the turtlebot in the simulation environment based on the sensor input and actuator output
  * qLearningModule.py - Q learning module source code that predicts the action based on Q values. Q table has been used for this module
2. Report.pdf - Description of the project, approach and the system implementation along with the results

## Running the tests

### Execution using point robot simulator
1. Open a terminal and goto codes folder.
2. Source the indoor_qlearning ROS package:
source devel/setup.bash
3. Execute the following command:
roslaunch indoor_qlearning qLearnPointRobot.launch

The program executes with the optimal policy being generated for a 10x10 gid. At the end of the execution, the path generation visualization is shown.

### Execution with turtlebot
1. Open a terminal and goto codes folder.
2. Source the indoor_qlearning ROS package:
source devel/setup.bash
3. Execute the following command:
roslaunch indoor_qlearning qlearn.launch

The program executes showing the win percentage Results

## Authors

**Janakiraman Kirthivasan** - *Initial work* - [jkvasan7692](https://github.com/jkvasan7692)
**Rama Prashanth** - *Initial work* - [ramaprashanth](https://github.com/ramaprashanth)

## Results link
The results of the project can be found in the following link:
https://drive.google.com/open?id=1BX9xXawbQ8oi-33Z9jE6Ps7tH0FSTobX
