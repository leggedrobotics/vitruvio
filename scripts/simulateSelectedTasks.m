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
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    if vitruvianBipedWalkOnSpot
        dataSelection = 'vitruvianBipedWalkOnSpot'; 
        classSelection = 'vitruvianBiped';
        task = 'walkOnSpot';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task,saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end

    if vitruvianBipedPushup
        dataSelection = 'vitruvianBipedPushup'; 
        classSelection = 'vitruvianBiped';
        task = 'pushup';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task,saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if universalTrot
        dataSelection = 'universalTrot'; 
        classSelection = 'universal';
        task = 'trot';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task,saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if universalStairs
        dataSelection = 'universalStairs'; 
        classSelection = 'universal';
        task = 'stairs';
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if speedyGallop
        dataSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        task = 'gallop';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if speedyStairs
        dataSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        task = 'stairs';      
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if massivoWalk
        dataSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        task = 'walk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if massivoStairs
        dataSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        task = 'stairs';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if centaurWalk
        dataSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        task = 'walk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if centaurStairs
        dataSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        task = 'stairs';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    if miniPronk
        dataSelection = 'miniPronk'; 
        classSelection = 'mini';
        task = 'pronk';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end

    if ANYmalTrot
        dataSelection = 'ANYmalTrot'; 
        classSelection = 'ANYmal';
        task = 'trot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalSlowTrotAccurateMotion
        dataSelection = 'ANYmalSlowTrotAccurateMotion'; 
        classSelection = 'ANYmal';
        task = 'slowTrotAccurateMotion';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalSlowTrot2
        dataSelection = 'ANYmalSlowTrot2'; 
        classSelection = 'ANYmal';
        task = 'slowTrot2';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalFlyingTrot
        dataSelection = 'ANYmalFlyingTrot'; 
        classSelection = 'ANYmal';
        task = 'flyingTrot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalTrotVersatilityStep
        dataSelection = 'ANYmalTrotVersatilityStep'; 
        classSelection = 'ANYmal';
        task = 'trotVersatilityStep';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalBearTrot
        dataSelection = 'ANYmalBearTrot'; 
        classSelection = 'ANYmalBear';
        task = 'trot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end

    if ANYmalBearTrot2
        dataSelection = 'ANYmalBearTrot2'; 
        classSelection = 'ANYmalBear';
        task = 'trot2';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end    
    
    if ANYmalBearTrot3
        dataSelection = 'ANYmalBearTrot3'; 
        classSelection = 'ANYmalBear';
        task = 'trot3';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end

    if ANYmalBearElongatedTrot
        dataSelection = 'ANYmalBearElongatedTrot'; 
        classSelection = 'ANYmalBear';
        task = 'elongatedTrot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalBearTrotSwing3
        dataSelection = 'ANYmalBearTrotSwing3'; 
        classSelection = 'ANYmalBear';
        task = 'trotSwing3';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalBearTrotSwing5
        dataSelection = 'ANYmalBearTrotSwing5'; 
        classSelection = 'ANYmalBear';
        task = 'trotSwing5';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalBearSlowTrot
        dataSelection = 'ANYmalBearSlowTrot'; 
        classSelection = 'ANYmalBear';
        task = 'slowTrot';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end

    if ANYmalBearSlowTrotIntermediateTorque
        dataSelection = 'ANYmalBearSlowTrotIntermediateTorque'; 
        classSelection = 'ANYmalBear';
        task = 'slowTrotIntermediateTorque';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
    
    if ANYmalBearFlyingTrot2
        dataSelection = 'ANYmalBearFlyingTrot2'; 
        classSelection = 'ANYmalBear';
        task = 'flyingTrot2';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
        
    
    if defaultHopperHop
        dataSelection = 'defaultHopperHop'; 
        classSelection = 'defaultHopper';
        task = 'hop';        
        results.(classSelection).(task)(i) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, runOptimization, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint);        
        results.(classSelection).(task)(i).metaParameters = orderfields(results.(classSelection).(task)(i).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(i).(EEselection) = orderfields(results.(classSelection).(task)(i).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection), task, optimizeLeg, saveFiguresToPDF);        
    end
end