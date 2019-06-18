clear;
close all;

%% Data extraction
% if true, the motion is segmented into individual steps which are averaged
% to create an average cycle. This works well when the motion is very cyclical.
% If false the individual steps are not averaged.
averageStepsForCyclicalMotion = true; 

%% Toggle leg properties, visualization and optimization functions
% number of links from 2 to 4. [thigh, shank, foot, phalanges]
linkCount = 2;
configSelection = 'X'; % X or M
actuateJointsDirectly = false;
% specify hip orientation
hipParalleltoBody = true; % if false, hip link is perpendicular to body x

%% AFE and DFE heuristics
% When no heuristic is specified, the angle of the final joint closest to 
% the starting q0 is obtained.
heuristic.torqueAngle.thetaLiftoff_des = pi/2; % specify desired angle between final link and horizonal at liftoff
heuristic.torqueAngle.kTorsionalSpring = 30; % spring constant for torsional spring at final joint [Nm/rad]
heuristic.torqueAngle.apply = true;

%% Toggle trajectory plots and initial design viz
viewVisualization = false; % initial leg design tracking trajectory plan
numberOfLoopRepetitions = 1; % number of steps visualized for leg motion
viewTrajectoryPlots = false;

%% Toggle optimization for each leg
runOptimization = false;
viewOptimizedLegPlot = false;
optimizeLF = true; 
optimizeLH = false; 
optimizeRF = false; 
optimizeRH = false;

%% Set optimization properties
% toggle visualization 
optimizationProperties.viz.viewVisualization = true;
optimizationProperties.viz.numberOfCyclesVisualized = 1;
optimizationProperties.viz.displayBestCurrentLinkLengths = true; % display chart while running ga

% Set number of generations and population size
optimizationProperties.options.maxGenerations = 20;
optimizationProperties.options.populationSize = 20;

% Set weights for fitness function terms
optimizationProperties.penaltyWeight.totalSwingTorque  = 0;
optimizationProperties.penaltyWeight.totalStanceTorque = 0;
optimizationProperties.penaltyWeight.totalTorque       = 0;
optimizationProperties.penaltyWeight.totalTorqueHFE    = 0;
optimizationProperties.penaltyWeight.swingTorqueHFE    = 0;
optimizationProperties.penaltyWeight.totalqdot         = 0;
optimizationProperties.penaltyWeight.totalPower        = 0; % only considers power terms > 0
optimizationProperties.penaltyWeight.totalEnergy       = 1;
optimizationProperties.penaltyWeight.maxTorque         = 0;
optimizationProperties.penaltyWeight.maxqdot           = 0;
optimizationProperties.penaltyWeight.maxPower          = 0; % only considers power terms > 0
optimizationProperties.penaltyWeight.maximumExtension  = true; % large penalty incurred if leg extends beyond allowable amount
optimizationProperties.allowableExtension              = 0.8; % penalize extension above this ratio of total possible extension

% Set bounds for link lengths as multipliers of initial values
if linkCount == 2
    optimizationProperties.bounds.upperBoundMultiplier = [1.5,   2,   2,  -1.5]; % [hip thigh shank hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.5, 0.3, 0.1, 1.5]; % [hip thigh shank hipAttachmentOffset]
end
if linkCount == 3
    optimizationProperties.bounds.upperBoundMultiplier = [1, 1.2, 1.2, 1.2, -2]; % [hip thigh shank foot hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.1, 0.1, 0.1, 2]; % [hip thigh shank foot hipAttachmentOffset ]
end
if linkCount == 4
    optimizationProperties.bounds.upperBoundMultiplier = [1, 1.2, 1.2, 3, 3, -2]; % [hip thigh shank foot phalanges hipAttachmentOffset]
    optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.1, 0.1, 0.1, 0.1, 2]; % [hip thigh shank foot phalanges hipAttachmentOffset]
end

% impose limits on maximum joint torque, speed and power
% the values are defined in getActuatorLimits
imposeJointLimits.maxTorque = false;
imposeJointLimits.maxqdot   = false;
imposeJointLimits.maxPower  = false;

%% Toggle robots and tasks to be simulated and optimized
universalTrot   = false;
universalStairs = false;
speedyStairs    = false;
speedyGallop    = false;
massivoWalk     = false;
massivoStairs   = false;
centaurWalk     = false;
centaurStairs   = false;
miniPronk       = false;
ANYmalTrot      = false;
ANYmalSlowTrot  = true; % stance torque a high
ANYmalSlowTrotGoodMotionBadForce = true;
ANYmalSlowTrotOriginal = false; % stance torque good but motion not the same as in measured

numberOfRepetitions = 0; % number of times that leg is reoptimized

%% run the simulation
simulateSelectedTasks;

%% additional plotting features
plotOptimizedLeg.LF = false; plotOptimizedLeg.LH = false;  plotOptimizedLeg.RF = false; plotOptimizedLeg.RH = false; 
if runOptimization
    plotOptimizedLeg.LF = optimizeLF;
    plotOptimizedLeg.LH = optimizeLH;
    plotOptimizedLeg.RF = optimizeRF;
    plotOptimizedLeg.RH = optimizeRH;
end
% additional plotting available via:
% plotJointDataForAllLegs(classSelection, 'taskSelection', plotOptimizedLeg)
 % plotJointDataForAllLegs(ANYmal, 'slowTrot', plotOptimizedLeg);
fprintf('Done.\n');