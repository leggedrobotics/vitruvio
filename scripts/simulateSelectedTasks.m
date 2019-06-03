% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task

for i = 1:numberOfRepetitions+1
    fprintf('Optimization count: %3.0f \n', i)
    if universalTrot
        taskSelection = 'universalTrot'; 
        classSelection = 'universal';
        universal.trot(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if universalStairs
        taskSelection = 'universalStairs'; 
        classSelection = 'universal';
        universal.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if speedyGallop
        taskSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        speedy.gallop(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if speedyStairs
        taskSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        speedy.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if massivoWalk
        taskSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        massivo.walk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if massivoStairs
        taskSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        massivo.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if centaurWalk
        taskSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        centaur.walk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if centaurStairs
        taskSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        centaur.stairs(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
    if miniPronk
        taskSelection = 'miniPronk'; 
        classSelection = 'mini';
        mini.pronk(i) = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
    end
end