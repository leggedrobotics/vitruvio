% This script runs the simulation as per the toggles in main and returns
% the data for each robot and task in a struct robot.task
EEnames = ['LF'; 'RF'; 'LH'; 'RH']; % For robots with fewer than 4 legs, a subset of these end effectors are used.

possibleDataSetNames = fieldnames(dataSelection);
numberOfDataSets = size(possibleDataSetNames);
numberOfActiveDataSets = 0;
for j = 1:numberOfDataSets
    if dataSelection.(possibleDataSetNames{j})
        numberOfActiveDataSets = numberOfActiveDataSets+1;
        activeDataSet(numberOfActiveDataSets) = {possibleDataSetNames{j}};
    end
end

for optimizationNumber = 1:optimizationCount
    fprintf('Optimization count: %3.0f \n', optimizationNumber)
    for j = 1:numberOfActiveDataSets
        selectedDataSet = char(activeDataSet(j));
        index = strfind(selectedDataSet, '_');
        classSelection = selectedDataSet(1:index-1);
        task           = selectedDataSet(index+1:end);
        results.(classSelection).(task)(optimizationNumber) = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointDirectly, transmissionMethod, linkCount, optimizeLeg, optimizationProperties, selectedDataSet, classSelection, configSelection, hipParalleltoBody, legCount, task, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, payload);        
        results.(classSelection).(task)(optimizationNumber).metaParameters = orderfields(results.(classSelection).(task)(optimizationNumber).metaParameters);        
        for k = 1:legCount
            EEselection = EEnames(k,:);
            results.(classSelection).(task)(optimizationNumber).(EEselection) = orderfields(results.(classSelection).(task)(optimizationNumber).(EEselection));
        end
        generatePlots(viewPlots, results.(classSelection).(task)(optimizationNumber), task, optimizeLeg, saveFiguresToPDF);    
        runVisualizationScripts(robotVisualization, optimizationProperties, results.(classSelection).(task)(optimizationNumber));
    end 
end