% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task
EEnames = ['LF'; 'RF'; 'LH'; 'RH']; % For robots with fewer than 4 legs, a subset of these end effectors are used.

for optimizationNumber = 1:optimizationCount
    fprintf('Optimization count: %3.0f \n', optimizationNumber)
    
    %%% Add your data file here %%%
    if yourTrajectoryData % toggle on or off in main
        dataSelection = 'yourTrajectoryData'; 
        classSelection = 'yourRobotClass';
        task = 'yourRobotTask';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if vitruvianBipedPushupSquat
        dataSelection = 'vitruvianBipedPushupSquat'; 
        classSelection = 'vitruvianBiped';
        task = 'pushupSquat';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
   
    if vitruvianBipedHop
        dataSelection = 'vitruvianBipedHop'; 
        classSelection = 'vitruvianBiped';
        task = 'hop';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if vitruvianBipedFastWalk
        dataSelection = 'vitruvianBipedFastWalk'; 
        classSelection = 'vitruvianBiped';
        task = 'fastWalk';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if universalTrot
        dataSelection = 'universalTrot'; 
        classSelection = 'universal';
        task = 'trot';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if universalStairs
        dataSelection = 'universalStairs'; 
        classSelection = 'universal';
        task = 'stairs';
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if speedyGallop
        dataSelection = 'speedyGallop'; 
        classSelection = 'speedy';
        task = 'gallop';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
        
    if speedyStairs
        dataSelection = 'speedyStairs'; 
        classSelection = 'speedy';
        task = 'stairs';      
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
        
    if massivoWalk
        dataSelection = 'massivoWalk'; 
        classSelection = 'massivo';
        task = 'walk';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
        
    if massivoStairs
        dataSelection = 'massivoStairs'; 
        classSelection = 'massivo';
        task = 'stairs';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
        
    if centaurWalk
        dataSelection = 'centaurWalk'; 
        classSelection = 'centaur';
        task = 'walk';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if centaurStairs
        dataSelection = 'centaurStairs'; 
        classSelection = 'centaur';
        task = 'stairs';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
        
    if miniPronk
        dataSelection = 'miniPronk'; 
        classSelection = 'mini';
        task = 'pronk';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearPushup
        dataSelection = 'ANYmalBearPushup'; 
        classSelection = 'ANYmalBear';
        task = 'pushup';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearFastTrot
        dataSelection = 'ANYmalBearFastTrot'; 
        classSelection = 'ANYmalBear';
        task = 'fastTrot';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearFastTrotExtendedxNom
        dataSelection = 'ANYmalBearFastTrotExtendedxNom'; 
        classSelection = 'ANYmalBear';
        task = 'fastTrotExtendedxNom';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearFastTrotExtendedxNom2
        dataSelection = 'ANYmalBearFastTrotExtendedxNom2'; 
        classSelection = 'ANYmalBear';
        task = 'fastTrotExtendedxNom2';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearFastTrotExtendedxNom3
        dataSelection = 'ANYmalBearFastTrotExtendedxNom3'; 
        classSelection = 'ANYmalBear';
        task = 'fastTrotExtendedxNom3';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if ANYmalBearSlowTrot
        dataSelection = 'ANYmalBearSlowTrot'; 
        classSelection = 'ANYmalBear';
        task = 'slowTrot';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
    
    if defaultHopperHop
        dataSelection = 'defaultHopperHop'; 
        classSelection = 'defaultHopper';
        task = 'hop';        
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, robotVisualization, linkCount, optimizeLeg, optimizationProperties, dataSelection, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for j = 1:legCount
            EEselection = EEnames(j,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end
end