clear;
close all;

%% Data extraction
% if averageStepsForCyclicalMotion is true, the motion is segmented into individual steps which are averaged
% to create an average cycle. This works well when the motion is very cyclical.
% If false the individual steps are not averaged. This should be selected
% when the generated motion is irregular and highly cyclical.
dataExtraction.averageStepsForCyclicalMotion = true; 
dataExtraction.allowableDeviation = 0.05; % [m] Deviation between neighbouring points. If the deviation is larger, additional points are interpolated.

%% Toggle leg properties: leg count, link count, configuration, direct/remote joint actuation, spider/serial leg
legCount  = 1;                  % Accepts values from 1 to 4.
linkCount = 2;                  % Accepts values from 2 to 4. [thigh, shank, foot, phalanges]. Hip link connects HAA and HFE but is not included in link count.
configSelection = 'X';          % X or M
actuateJointsDirectly = true;   % If true, actuators are positioned in each joint which contributes to leg mass and inertia. If false, there is no actuator mass at joints.

% Specify hip orientation
% if true: Serial configuration. Offset from HAA to HFE parallel to the body as with ANYmal 
% if false: Spider configuration. Hip link is perpendicular to body length.
hipParalleltoBody = true;

%% AFE and DFE heuristics (for 3 and 4 link legs)
% The heuristic computes the final joint angle (AFE or DFE) as a 
% deformation proportional to torque. For a four link leg, the thigh and
% foot are maintained parallel.
heuristic.torqueAngle.apply = true; % Choose whether to apply the heuristic.
heuristic.torqueAngle.thetaLiftoff_des = pi/2; % Specify desired angle between final link and horizonal at liftoff. If the desired angle is impossible for the given link lengths, the closest feasible angle is obtained.
heuristic.torqueAngle.kTorsionalSpring = 20; % Spring constant for torsional spring at final joint [Nm/rad]

%% Toggle trajectory plots and initial design viz
viewVisualization            = false; % initial leg design tracking trajectory plan
numberOfStepsVisualized      = 1;     % number of steps visualized for leg motion
viewPlots.motionData         = true;  % CoM position, speed. EE position and forces.
viewPlots.rangeOfMotionPlots = true; % range of motion of leg for given link lengths and angle limits
viewPlots.efficiencyMap      = true; % actuator operating efficiency map
viewPlots.jointDataPlot      = true; % angle, speed, torque, power, energy data
viewPlots.metaParameterPlot  = true; % design parameters and key results plotted as pie charts

%% Select a .mat trajectory data file to be simulated and optimized
% Select from the below options or import a new data .mat set using the
% importMotionData script

%%% Add your trajectory data file here %%%
yourTrajectoryData = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

universalTrot   = true;
universalStairs = false;
speedyStairs    = false;
speedyGallop    = false;
massivoWalk     = false;
massivoStairs   = false;
centaurWalk     = false;
centaurStairs   = false;
miniPronk       = false;
ANYmalTrot      = false;
ANYmalSlowTrotAccurateMotion = false;
defaultHopperHop = false;
ANYmalSlowTrot2  = false;

numberOfRepetitions = 0; % Number of times that leg is reoptimized. This allows for an easy check if the same optimal solution is found each time the optimization is run.

%% Select actuators for each joint
% Select from: {ANYdrive, Neo, RobotDrive, other} or add a new actuator in
% getActuatorProperties
actuatorSelection.HAA = 'ANYdrive'; 
actuatorSelection.HFE = 'ANYdrive'; 
actuatorSelection.KFE = 'ANYdrive';
actuatorSelection.AFE = 'ANYdrive'; 
actuatorSelection.DFE = 'ANYdrive'; 

%% Toggle optimization for each leg
runOptimization = true; 
% select which legs are to be optimized
optimizeLeg.LF = true; 
optimizeLeg.RF = false; 
optimizeLeg.LH = false; 
optimizeLeg.RH = false;

%% Set optimization properties
% toggle visualization 
optimizationProperties.viz.viewVisualization = true;
optimizationProperties.viz.numberOfCyclesVisualized = 1;
optimizationProperties.viz.displayBestCurrentLinkLengths = true; % display chart of current best leg design parameters while running ga

% Set number of generations and population size
optimizationProperties.options.maxGenerations = 20;
optimizationProperties.options.populationSize = 20;

% Impose limits on maximum joint torque, speed and power
% the values are defined in getActuatorProperties. A penalty term is incurred
% for violations of these limits.
imposeJointLimits.maxTorque = true;
imposeJointLimits.maxqdot   = false;
imposeJointLimits.maxPower  = true;

% Set weights for fitness function terms. Total means summed over all
% joints in the leg.
optimizationProperties.penaltyWeight.totalSwingTorque  = 0;
optimizationProperties.penaltyWeight.totalStanceTorque = 0;
optimizationProperties.penaltyWeight.totalTorque       = 0;
optimizationProperties.penaltyWeight.totalTorqueHFE    = 0;
optimizationProperties.penaltyWeight.swingTorqueHFE    = 0;
optimizationProperties.penaltyWeight.totalqdot         = 0;
optimizationProperties.penaltyWeight.totalPower        = 0;     % only considers power terms > 0
optimizationProperties.penaltyWeight.totalMechEnergy   = 0;
optimizationProperties.penaltyWeight.totalElecEnergy   = 0;
optimizationProperties.penaltyWeight.averageEfficiency = 0;     % Maximizes average efficiency (even though this could increase overall energy use)
optimizationProperties.penaltyWeight.maxTorque         = 1;
optimizationProperties.penaltyWeight.maxqdot           = 0;
optimizationProperties.penaltyWeight.maxPower          = 0;     % only considers power terms > 0
optimizationProperties.penaltyWeight.antagonisticPower = 0;     % seeks to minimize antagonistic power which improves power quality
optimizationProperties.penaltyWeight.maximumExtension  = true;  % large penalty incurred if leg extends beyond allowable amount
optimizationProperties.allowableExtension              = 0.9;   % [0 1] penalize extension above this ratio of total possible extension

% Set bounds for link lengths as multipliers of initial values
if linkCount == 2
    optimizationProperties.bounds.upperBoundMultiplier = [2,   0.5,   0.5,  2]; % [hip thigh shank hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.5,  2,  2,  0.5]; % [hip thigh shank hipAttachmentOffset]
end
if linkCount == 3
    optimizationProperties.bounds.upperBoundMultiplier = [2, 2, 2, 1.2, -2]; % [hip thigh shank foot hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.5, 0.5, 0.1, 0.1, 2]; % [hip thigh shank foot hipAttachmentOffset ]
end
if linkCount == 4
    optimizationProperties.bounds.upperBoundMultiplier = [1, 1.2, 1.2, 1, 1, -1]; % [hip thigh shank foot phalanges hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.8, 0.8, 1, 1, 1]; % [hip thigh shank foot phalanges hipAttachmentOffset]
end

%% run the simulation
if ~runOptimization % if optimization turned off, set values to zero.
    optimizeLeg.LF = 0; optimizeLeg.RF = 0; optimizeLeg.LH = 0; optimizeLeg.RH = 0;
end
simulateSelectedTasks;
fprintf('Done.\n');