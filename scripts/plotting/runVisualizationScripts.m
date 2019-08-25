function runVisualizationScripts(robotVisualization, optimizationProperties, results, classSelection, task)
EEnames = results.(classSelection).(task).basicProperties.EEnames;
config = results.(classSelection).(task).basicProperties.configSelection;

%% Nominal robot
  if robotVisualization.view 
    fprintf('Visualizing nominal robot. \n');
    optimized = false;
    fileName = strcat('Nominal_', classSelection, '_',task, '_', config, '.gif'); % for gif of motion
    
    if robotVisualization.plotAllLegs
        vizIndex = 1;
    elseif robotVisualization.plotOneLeg
        vizIndex = results.(classSelection).(task).basicProperties.legCount;
    end
    
    for i = 1:vizIndex
        EEselection = EEnames(i,:);
        visualizeRobot(results, classSelection, task, EEselection, fileName, robotVisualization, optimized);
    end
  end
    
%% Optimized robot
    if results.(classSelection).(task).optimizationProperties.runOptimization && optimizationProperties.viz.viewVisualization
        fprintf('Visualizing optimized robot. \n');
        optimized = true;
        fileName = strcat('Optimized_', classSelection, '_',task, '_', config, '.gif'); % for gif of motion
        if robotVisualization.plotAllLegs
            vizIndex = 1;
        elseif robotVisualization.plotOneLeg
            vizIndex = results.(classSelection).(task).basicProperties.legCount;
        end

        for i = 1:vizIndex
            EEselection = EEnames(i,:);
            visualizeRobot(results, classSelection, task, EEselection, fileName, robotVisualization, optimized);
        end
    end
  end