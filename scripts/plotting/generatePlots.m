function [] = generatePlots(viewPlots, data, task, optimizeLeg, saveFiguresToPDF)
%% Create title page for the pdf

% classSelection = data.(task).basicProperties.classSelection;
% figure('name', 'Title Slide', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
% set(gcf,'color','w')
% text(0.5,0.5,['Results of vitruvio simulation for ', classSelection, ' ', task], 'FontSize', 22,'HorizontalAlignment', 'center');
% set(gca, 'visible', 'off', 'xtick', []);
% if saveFiguresToPDF
%     export_fig results.pdf -nocrop -append
% end

%% Motion plot
if viewPlots.motionData
    fprintf('Plotting motion data. \n')
    plotMotionData(data, task, saveFiguresToPDF);
end
%% Range of motion plot
if viewPlots.rangeOfMotionPlots
    fprintf('Plotting range of motion. \n')    
    plotRangeOfMotion(data, task, saveFiguresToPDF);
end
%% Joint data
if viewPlots.jointDataPlot
    fprintf('Plotting actuator speed, torque, power and energy data. \n')    
    plotJointDataForAllLegs(data, task, [], [], optimizeLeg, saveFiguresToPDF) % Can optionally plot a second data set on the same axes
end
%% Meta parameters
if viewPlots.metaParameterPlot
    fprintf('Plotting meta parameters. \n')
    plotMetaParameters(data, task, saveFiguresToPDF)
end
%% Motor efficiency
if viewPlots.efficiencyMap
    fprintf('Plotting motor efficiency map. \n')    
    plotEfficiencyMapWithOperatingPoints(data, task, saveFiguresToPDF)
end
%% Generate data table in command window
%tabulateResults(data, task)