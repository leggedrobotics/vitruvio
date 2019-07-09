# vitruvio

## Description

Vitruvio is a method for rapid leg design optimization for legged robots. The framework loads a trajectory plan consisting of a center of mass position and orientation as well as end effector positions and forces over time. The user can make several high level design decisions such as direct/remote actuation of joints, number of links, leg configuration and actuator selection. The joint angles, speeds, torques, power and energy necessary to track the input trajectory are then computed. Optionally, a set of low level design parameters are then optimized using a genetic algorithm optimizer to reduce a user-specified cost funtion.

The input trajectories have generally been generated using the Towr trajectory optimizer: https://github.com/ethz-adrl/towr 

Towr allows for quick computation of trajectories for different tasks using a small set of robot design parameters and as such is well suited to aiding in simulation in the early design stages.


## Installation

Clone the vitruvio repository and open in Matlab. The following Matlab Toolboxes are required:

Robotic System Toolbox, Global Optimization Toolbox

## Use

For an introduction vitruvio, open the main script and review the different options which can be toggled on/off. These include high level design decisions, visualization options and selection of robots to be optimized. To add your own robot, use the importMotionData script which converts rosbags into a .mat file for use in vitruvio. Be sure to also add your robot's properties need to be added to getQuadrupedProperties and add the class and task names to simulateSelectedTasks.