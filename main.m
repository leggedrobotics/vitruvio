clear;
close all;

%% Toggle leg properties, visualization and optimization functions
% number of links from 2 to 4. [thigh, shank, foot, phalanges]
linkCount = 4;

% Toggle trajectory plots and initial design viz
viewVisualization = false; 
numberOfLoopRepetitions = 1;
viewTrajectoryPlots = false;

% Toggle optimization for each leg
runOptimization = true;
viewOptimizedLegPlot = true;
optimizeLF = true; 
optimizeLH = false; 
optimizeRF = false; 
optimizeRH = false;

%% set optimization properties
% toggle visualization 
optimizationProperties.viz.viewVisualization = false;
optimizationProperties.viz.displayBestCurrentLinkLengths = false; % display chart while running ga

% set number of generations and population size
optimizationProperties.options.maxGenerations = 5;
optimizationProperties.options.populationSize = 5;

% set weights for fitness function terms
optimizationProperties.penaltyWeight.totalTorque =   1;
optimizationProperties.penaltyWeight.totalqdot =     0;
optimizationProperties.penaltyWeight.totalPower =    0;
optimizationProperties.penaltyWeight.maxTorque =     0;
optimizationProperties.penaltyWeight.maxqdot =       0;
optimizationProperties.penaltyWeight.maxPower =      0;
optimizationProperties.penaltyWeight.trackingError = 100000;

% set bounds for link lengths as multipliers of initial values
optimizationProperties.bounds.upperBoundMultiplier = [1, 3, 3]; % [hip thigh shank]
optimizationProperties.bounds.lowerBoundMultiplier = [1, 0.5, 0.5]; % [hip thigh shank]
if linkCount == 3
    optimizationProperties.bounds.upperBoundMultiplier = [3, 3, 3, 3]; % [hip thigh shank]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.3, 0.5, 0.5, 0.5]; % [hip thigh shank]
end
if linkCount == 4
    optimizationProperties.bounds.upperBoundMultiplier = [3, 3, 3, 3, 3]; % [hip thigh shank]
    optimizationProperties.bounds.lowerBoundMultiplier = [0.3, 0.5, 0.5, 0.5, 0.5]; % [hip thigh shank]
end

%% Toggle robots and tasks to be simulated and optimized
universalTrot = true;
universalStairs = true;
speedyStairs = false;
speedyGallop = false;
massivoWalk = false;
massivoStairs = false;
centaurWalk = false;
centaurStairs = true;
miniPronk = false;
configSelection = 'M'; % this feature needs to be reworked 

%% run the simulation as per the above selections
if universalTrot
    taskSelection = 'universalTrot'; 
    robotSelection = 'universal';
    universal.trot = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if universalStairs
    taskSelection = 'universalStairs'; 
    robotSelection = 'universal';
    universal.stairs = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if speedyGallop
    taskSelection = 'speedyGallop'; 
    robotSelection = 'speedy';
    speedy.gallop = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if speedyStairs
    taskSelection = 'speedyStairs'; 
    robotSelection = 'speedy';
    speedy.stairs = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if massivoWalk
    taskSelection = 'massivoWalk'; 
    robotSelection = 'massivo';
    massivo.walk = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if massivoStairs
    taskSelection = 'massivoStairs'; 
    robotSelection = 'massivo';
    massivo.stairs = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if centaurWalk
    taskSelection = 'centaurWalk'; 
    robotSelection = 'centaur';
    centaur.walk = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if centaurStairs
    taskSelection = 'centaurStairs'; 
    robotSelection = 'centaur';
    centaur.stairs = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end
if miniPronk
    taskSelection = 'miniPronk'; 
    robotSelection = 'mini';
    mini.pronk = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
end