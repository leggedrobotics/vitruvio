% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task
EEnames = ['LF'; 'RF'; 'LH'; 'RH']; % For robots with fewer than 4 legs, a subset of these end effectors are used.

for i = 1:numberOfRepetitions+1
    fprintf('Optimization count: %3.0f \n', i)
    
    %%% Add your data file here %%%
    if yourTrajectoryData % toggle on or off in main
        dataSelection = 'yourTrajectoryData'; 
        classSelection = 'yourRobotClass';
        task = 'yourRobotTask';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if universalTrot
        dataSelection = 'universalTrot'; 
        classSelection = 'universal';
        task = 'trot';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    
    if universalStairs
        dataSelection = 'universalStairs'; 
        classSelection = 'universal';
        task = 'stairs';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    
    if speedyGallop
        dataSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        task = 'gallop';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if speedyStairs
        dataSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        task = 'stairs';      
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if massivoWalk
        dataSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        task = 'walk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if massivoStairs
        dataSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        task = 'stairs';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if centaurWalk
        dataSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        task = 'walk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if centaurStairs
        dataSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        task = 'stairs';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
        
    if miniPronk
        dataSelection = 'miniPronk'; 
        classSelection = 'mini';
        task = 'pronk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end

    if ANYmalTrot
        dataSelection = 'ANYmalTrot'; 
        classSelection = 'ANYmal';
        task = 'trot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    
    if ANYmalSlowTrotAccurateMotion
        dataSelection = 'ANYmalSlowTrotAccurateMotion'; 
        classSelection = 'ANYmal';
        task = 'slowTrotAccurateMotion';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    
    if ANYmalSlowTrot2
        dataSelection = 'ANYmalSlowTrot2'; 
        classSelection = 'ANYmal';
        task = 'slowTrot2';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
    
    if defaultHopperHop
        dataSelection = 'defaultHopperHop'; 
        classSelection = 'defaultHopper';
        task = 'hop';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg);        
    end
end