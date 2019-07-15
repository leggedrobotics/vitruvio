# _vitruvio_

## Description

_vitruvio_ is a framework for rapid leg design optimization for legged robots. The purpose of the simulation framework is to guide the early stages of legged robot design. The end effectors track an input trajectory and the necessary joint speed, torque, power and energy for the tracking is computed. These values are subject to a set of customizable user design selections in the form of toggle switches. 
Optionally, a set of low level design parameters are then optimized using a genetic algorithm optimizer to reduce a user-specified cost funtion.

The framework relies on first importing trajectory data consisting of a center of mass position and orientation as well as end effector positions and forces over time. The input trajectories have generally been generated using the Towr trajectory optimizer: https://github.com/ethz-adrl/towr 

Towr allows for quick computation of trajectories for different tasks using a small set of robot design parameters and as such is well suited to aiding in simulation in the early design stages. 


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/oqQyCcB4QVo/0.jpg)](https://www.youtube.com/watch?v=oqQyCcB4QVo)

## Features

:white_check_mark: Computation of joint speed, torque, power and energy required to track an input trajectory.

:white_check_mark: Highly versatile to different user design choices and robot properties allowing for quick comparison of high level design decisions.

   * Direct actuation with actuator placement in the joints or remote actuation with actuators placed in the body.
   * Leg quantity ranging from one to four legs.
   * X or M leg layout.
   * Link quantity ranging from two to four links.
   * Link lengths and density.
   * Actuator selection from a list of existing actuators including ANYdrive and Neo.
   * The input trajectory can be tracked in its entirety, only for a sample of the entire motion, or averaged into a single representative step cycle.

:white_check_mark: Motion visualization.

:white_check_mark: Meta parameter computation including:

   * Cost of transport.
   * Range of joint angles required for motion tracking.
   * Maximum values of joint speed, torque, and power.
   * Energy consumed per step cycle.

:white_check_mark: Design parameter optimization.

   * Link lengths
   
     In future versions additionally:
   
   * Torsional spring parameters for spring at ankle flexion/extension joint
   * Leg layout
   * Link quantity
   * Actuator placement

:white_check_mark: Result plotting.

:white_check_mark: Facilitated addition of new robot classes and motion tasks.


## Installation

Clone the _vitruvio_ repository and open it in Matlab. Right click on the _vitruvio_ folder and add the folder and subfolders to path. The following Matlab Toolboxes are required:

   * Robotic System Toolbox
   * Global Optimization Toolbox
   
These can be added using the Add-Ons button in the Matlab Home tab.

## Usage

The intended work flow is as follows:
   
   1. High level robot design decisions are made including quantity of legs, robot mass, center of mass height and end effector positions in nominal stance.
   2. Based on these decisions, the inertia tensor for the robot about its center of mass are computed. This is readily done using CAD and in the future a template for NX will be created to facilitate this process for _vitruvio_ when detailed CAD data for the robot is available.
   3. These design parameters and inertia tensors are input into a new robot model in Towr and the motion is simulated for a given task by specifying gait, goal position, duration and terrain.
   4. The simulated data is imported into _vitruvio_.
   5. The user makes high level design decisions in vitruvio and obtains the required joint speed, torque, power, energy and meta parameters such as cost of transport. If applicable, the optimized leg design parameters are also returned along with comparison of the results for the nominal and optimized leg. These are all recorded in a structure named __results__.
   6. The high level design decisions and cost terms are adapted as desired.
   7. A file titled 'results.pdf' is automatically generated containing all the figures created during the simulation. This should be renamed and saved along with the output of the command window to fully record all of the results and selections. 

The figures are exported to a pdf file using: https://github.com/altmany/export_fig.

## Example

To familiarize yourself with _vitruvio_, open the __testExample.m__ script in the vitruvio folder and review the different options which can be toggled on/off. These include high level design decisions, visualization options and selection of robots to be optimized. 
The toggles have been set to simulate the universal robot class performing a trot motion using the nominal robot properties read in from the __getQuadrupedProperties.m__ script. 

Run __testExample.m__ and observe the result. The motion is visualized and several plots are generated to show the trajectory and joint data. The values are saved in a structure named after the robot class and task; in this case _universal.trot_. This structure contains all the relevant input and output data of the simulation.

### Optimization 

Now try running the optimization by setting _runOptimization = true_. This will run the genetic algorithm optimization for the link lengths within the specified upper and lower bounds. The default population size and number of generations are both 10 but can be increased to improve the result of the optimization. While the optimization runs, the penalty value of the current best design is shown in the command window. This has been normalized such that values < 1 are an improvement on the nominal design while values > 1 generally indicate worse performance than the nominal design. If the penalty is > 1 this can also indicate that a soft constraint has been violated. Soft constraints violations include joint positions below ground and actuator torque, speed and power limits. Theses are found in __computePenalty.m__.

The cost function is a sum of penalty terms which can be included in the cost function by setting the respective _optimizationProperties.penaltyWeight.penaltyTerm = 1_ or removed from the cost function by setting the term to zero. The maximum allowable extension of the leg can also be specified by editing _optimizationProperties.allowableExtension_. This is shown as a ratio of the total possible leg extension and penalizes leg extension beyond this value.

Furthermore, the maximum joint speed, torque and power of the actuators can be imposed as soft constraints in the cost function by setting them to true in _imposeJointLimits_. The joint limits are loaded in for the actuators which have been selected.

Play around with the different toggle options to understand the different degrees of freedom available to the user.

Once you're finished experimenting in __testExample.m__ open up the __main.m__ script where you can get started running your own simulations.

### Adding your own robot 

To add your own robot and task, first the trajectory data must be input into _vitruvio_. We recommend using Towr for the trajectory planning but any trajectory set consisting of base pose, end effector position and forces is sufficient. If using Towr, the first step is to run the __importMotionData.m__ script which allows you to select a rosbag to read the data from and a name to save your data into a .mat file. Remember to also set the number of legs here.
If using an alternative method for trajectory generation, ensure that the naming convention for the resulting .mat file is consistent with that created by the __importMotionData.m__ script.


After importing the motion data, you will need to add your robot and task to the following scripts in _vitruvio_.

   1. main
   2. simulateSelectedTasks
   3. getRobotProperties
   4. optionally getSuggestedRemovalRatios 
   
Each of these scripts has a section at the top with a template to follow in adding your own robot and task. Similarly if you would like to add a new actuator, this can be done in __getActuatorProperties.m__ 

## License