% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task
EEnames = ['LF'; 'RF'; 'LH'; 'RH'];

for i = 1:numberOfRepetitions+1
    fprintf('Optimization count: %3.0f \n', i)
    
    if universalTrot
        dataSelection = 'universalTrot'; 
        classSelection = 'universal';
        task = 'trot';
        universal.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);
        universal.(task)(i).metaParameters = orderfields(universal.(task)(i).metaParameters);
        for j = 1:legCount
            EEselection = EEnames(j,:);
            universal.(task)(i).(EEselection) = orderfields(universal.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, universal, task); % classSelection, task
    end
    
    if universalStairs
        dataSelection = 'universalStairs'; 
        classSelection = 'universal';
        task = 'stairs';
        universal.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        universal.(task)(i).metaParameters = orderfields(universal.(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            universal.(task)(i).(EEselection) = orderfields(universal.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, universal, task);        
    end
    
    if speedyGallop
        dataSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        task = 'gallop';        
        speedy.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        speedy.(task)(i).metaParameters = orderfields(speedy.(task)(i).metaParameters);
        for j = 1:legCount
            EEselection = EEnames(j,:);
            speedy.(task)(i).(EEselection) = orderfields(speedy.(task)(i).(EEselection));
        end
          generatePlots(viewPlots, speedy, task);      
    end
        
    if speedyStairs
        dataSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        task = 'stairs';      
        speedy.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        speedy.(task)(i).metaParameters = orderfields(speedy.(task)(i).metaParameters);         
        for j = 1:legCount
            EEselection = EEnames(j,:);
            speedy.(task)(i).(EEselection) = orderfields(speedy.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, speedy, task);          
    end
        
    if massivoWalk
        dataSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        task = 'walk';        
        massivo.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        massivo.(task)(i).metaParameters = orderfields(massivo.(task)(i).metaParameters);                 
        for j = 1:legCount
            EEselection = EEnames(j,:);
            massivo.(task)(i).(EEselection) = orderfields(massivo.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, massivo, task);        
    end
        
    if massivoStairs
        dataSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        task = 'stairs';        
        massivo.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        massivo.(task)(i).metaParameters = orderfields(massivo.(task)(i).metaParameters);                         
        for j = 1:legCount
            EEselection = EEnames(j,:);
            massivo.(task)(i).(EEselection) = orderfields(massivo.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, massivo, task);        
    end
        
    if centaurWalk
        dataSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        task = 'walk';        
        centaur.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        centaur.(task)(i).metaParameters = orderfields(centaur.(task)(i).metaParameters);                                 
        for j = 1:legCount
            EEselection = EEnames(j,:);
            centaur.(task)(i).(EEselection) = orderfields(centaur.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, centaur, task);        
    end
        
    if centaurStairs
        dataSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        task = 'stairs';        
        centaur.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        centaur.(task)(i).metaParameters = orderfields(centaur.(task)(i).metaParameters);                                       
        for j = 1:legCount
            EEselection = EEnames(j,:);
            centaur.(task)(i).(EEselection) = orderfields(centaur.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, centaur, task);        
    end
        
    if miniPronk
        dataSelection = 'miniPronk'; 
        classSelection = 'mini';
        task = 'pronk';        
        mini.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        mini.(task)(i).metaParameters = orderfields(mini.(task)(i).metaParameters);                                               
        for j = 1:legCount
            EEselection = EEnames(j,:);
            mini.(task)(i).(EEselection) = orderfields(mini.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, mini, task);        
    end

    if ANYmalTrot
        dataSelection = 'ANYmalTrot'; 
        classSelection = 'ANYmal';
        task = 'trot';        
        ANYmal.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        ANYmal.(task)(i).metaParameters = orderfields(ANYmal.(task)(i).metaParameters);                                                       
        for j = 1:legCount
            EEselection = EEnames(j,:);
            ANYmal.(task)(i).(EEselection) = orderfields(ANYmal.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, ANYmal, task);        
    end
    
    if ANYmalSlowTrotAccurateMotion
        dataSelection = 'ANYmalSlowTrotAccurateMotion'; 
        classSelection = 'ANYmal';
        task = 'slowTrotAccurateMotion';        
        ANYmal.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);      
        ANYmal.(task)(i).metaParameters = orderfields(ANYmal.(task)(i).metaParameters);                                                              
        for j = 1:legCount
            EEselection = EEnames(j,:);
            ANYmal.(task)(i).(EEselection) = orderfields(ANYmal.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, ANYmal, task);        
    end
    
    if defaultHopperHop
        dataSelection = 'defaultHopperHop'; 
        classSelection = 'defaultHopper';
        task = 'hop';        
        defaultHopper.(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task);        
        defaultHopper.(task)(i).metaParameters = orderfields(defaultHopper.(task)(i).metaParameters);                                                              
        for j = 1:legCount
            EEselection = EEnames(j,:);
            defaultHopper.(task)(i).(EEselection) = orderfields(defaultHopper.(task)(i).(EEselection));
        end
        generatePlots(viewPlots, defaultHopper, task);        
    end
end