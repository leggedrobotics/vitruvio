% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task

for i = 1:numberOfRepetitions+1
    fprintf('Optimization count: %3.0f \n', i)
    if universalTrot
        taskSelection = 'universalTrot'; 
        robotSelection = 'universal';
        universal.trot(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if universalStairs
        taskSelection = 'universalStairs'; 
        robotSelection = 'universal';
        universal.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if speedyGallop
        taskSelection = 'speedyGallop'; 
        robotSelection = 'speedy';
        speedy.gallop(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if speedyStairs
        taskSelection = 'speedyStairs'; 
        robotSelection = 'speedy';
        speedy.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if massivoWalk
        taskSelection = 'massivoWalk'; 
        robotSelection = 'massivo';
        massivo.walk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if massivoStairs
        taskSelection = 'massivoStairs'; 
        robotSelection = 'massivo';
        massivo.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if centaurWalk
        taskSelection = 'centaurWalk'; 
        robotSelection = 'centaur';
        centaur.walk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if centaurStairs
        taskSelection = 'centaurStairs'; 
        robotSelection = 'centaur';
        centaur.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
    if miniPronk
        taskSelection = 'miniPronk'; 
        robotSelection = 'mini';
        mini.pronk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, robotSelection, configSelection);
    end
end