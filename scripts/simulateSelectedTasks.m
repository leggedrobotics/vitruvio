% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task
EEnames = ['LF'; 'RF'; 'LH'; 'RH'];

for i = 1:numberOfRepetitions+1
    fprintf('Optimization count: %3.0f \n', i)
    
    if universalTrot
        taskSelection = 'universalTrot'; 
        classSelection = 'universal';
        universal.trot(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            universal.trot(i).(EEselection) = orderfields(universal.trot(i).(EEselection));
        end
    end
    
    if universalStairs
        taskSelection = 'universalStairs'; 
        classSelection = 'universal';
        universal.stairs(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            universal.stairs(i).(EEselection) = orderfields(universal.stairs(i).(EEselection));
        end
    end
    
    if speedyGallop
        taskSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        speedy.gallop(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            speedy.gallop(i).(EEselection) = orderfields(speedy.gallop(i).(EEselection));
        end
    end
        
    if speedyStairs
        taskSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        speedy.stairs(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
          for j = 1:4
            EEselection = EEnames(j,:);
            speedy.stairs(i).(EEselection) = orderfields(speedy.stairs(i).(EEselection));
          end
    end
        
    if massivoWalk
        taskSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        massivo.walk(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            massivo.walk(i).(EEselection) = orderfields(massivo.walk(i).(EEselection));
        end
    end
        
    if massivoStairs
        taskSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        massivo.stairs(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            massivo.stairs(i).(EEselection) = orderfields(massivo.stairs(i).(EEselection));
        end
    end
        
    if centaurWalk
        taskSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        centaur.walk(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            centaur.walk(i).(EEselection) = orderfields(centaur.walk(i).(EEselection));
        end
    end
        
    if centaurStairs
        taskSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        centaur.stairs(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            centaur.stairs(i).(EEselection) = orderfields(centaur.stairs(i).(EEselection));
        end
    end
        
    if miniPronk
        taskSelection = 'miniPronk'; 
        classSelection = 'mini';
        mini.pronk(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            mini.pronk(i).(EEselection) = orderfields(mini.pronk(i).(EEselection));
        end
    end

    if ANYmalTrot
        taskSelection = 'ANYmalTrot'; 
        classSelection = 'ANYmal';
        ANYmal.trot(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            ANYmal.trot(i).(EEselection) = orderfields(ANYmal.trot(i).(EEselection));
        end
    end
    if ANYmalSlowTrot
        taskSelection = 'ANYmalSlowTrot'; 
        classSelection = 'ANYmal';
        ANYmal.slowTrot(i) = runDataExtractionAndOptScripts(actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody);
        for j = 1:4
            EEselection = EEnames(j,:);
            ANYmal.slowTrot(i).(EEselection) = orderfields(ANYmal.slowTrot(i).(EEselection));
        end
    end
end