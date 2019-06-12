clear;
close all;

%% Toggle leg properties, visualization and optimization functions
% number of links from 2 to 4. [thigh, shank, foot, phalanges]
linkCount = 2;
configSelection = 'X'; % X or M
actuateJointsDirectly = true;
% specify hip orientation
hipParalleltoBody = true; % if false, hip link is perpendicular to body x

% Toggle trajectory plots and initial design viz
viewVisualization = true; % initial leg design tracking trajectory plan
numberOfLoopRepetitions = 1; % number of steps visualized for leg motion
viewTrajectoryPlots = false;

% Toggle optimization for each leg
runOptimization = false;
viewOptimizedLegPlot = false;
optimizeLF = true; 
optimizeLH = false; 
optimizeRF = false; 
optimizeRH = false;

%% set optimization properties
% toggle visualization 
optimizationProperties.viz.viewVisualization = false;
optimizationProperties.viz.displayBestCurrentLinkLengths = false; % display chart while running ga

% set number of generations and population size
optimizationProperties.options.maxGenerations = 4;
optimizationProperties.options.populationSize = 5;

% set weights for fitness function terms
optimizationProperties.penaltyWeight.totalSwingTorque  = 0;
optimizationProperties.penaltyWeight.totalStanceTorque = 0;
optimizationProperties.penaltyWeight.totalTorque       = 1;
optimizationProperties.penaltyWeight.totalTorqueHFE    = 0;
optimizationProperties.penaltyWeight.swingTorqueHFE    = 0;
optimizationProperties.penaltyWeight.totalqdot         = 0;
optimizationProperties.penaltyWeight.totalPower        = 0; % only considers power terms > 0
optimizationProperties.penaltyWeight.maxTorque         = 0;
optimizationProperties.penaltyWeight.maxqdot           = 0;
optimizationProperties.penaltyWeight.maxPower          = 0; % only considers power terms > 0
optimizationProperties.penaltyWeight.maximumExtension  = true; % large penalty incurred if leg extends beyond allowable amount
optimizationProperties.allowableExtension              = 0.8; % penalize extension above this ratio of total possible extension

% set bounds for link lengths as multipliers of initial values
if linkCount == 2
    optimizationProperties.bounds.upperBoundMultiplier = [2, 2, 2]; % [hip thigh shank]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.1, 0.3, 0.3]; % [hip thigh shank]
end
if linkCount == 3
    optimizationProperties.bounds.upperBoundMultiplier = [1, 1.2, 1.2, 1.2]; % [hip thigh shank]
    optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.1, 0.1, 0.1]; % [hip thigh shank]
end
if linkCount == 4
    optimizationProperties.bounds.upperBoundMultiplier = [1, 1.2, 1.2, 3, 3]; % [hip thigh shank]
    optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.1, 0.1, 0.1, 0.1]; % [hip thigh shank]
end

%% Toggle robots and tasks to be simulated and optimized
universalTrot   = false;
universalStairs = true;
speedyStairs    = false;
speedyGallop    = false;
massivoWalk     = false;
massivoStairs   = false;
centaurWalk     = false;
centaurStairs   = false;
miniPronk       = false;
ANYmalTrot      = false;
ANYmalSlowTrot  = false;

numberOfRepetitions = 0; % number of times that leg is reoptimized

%% run the simulation
simulateSelectedTasks;
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
