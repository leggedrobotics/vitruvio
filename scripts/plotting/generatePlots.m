function [] = generatePlots(viewPlots, data, task, optimizeLeg)

%% Create title page for the pdf
classSelection = data.(task).basicProperties.classSelection;
figure('units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
text(0.5,0.5,['Results of vitruvio simulation for ', classSelection, ' ', task], 'FontSize', 22,'HorizontalAlignment', 'center');
set(gca, 'visible', 'off', 'xtick', []);
export_fig results.pdf -nocrop -append

%% Motion plot
if viewPlots.motionData
    plotMotionData(data, task);
end
%% Range of motion plot
if viewPlots.rangeOfMotionPlots
    plotRangeOfMotion(data, task);
end
%% Joint data
if viewPlots.jointDataPlot
    plotJointDataForAllLegs(data, task, [], [], optimizeLeg) % Can optionally plot a second data set on the same axes
end
%% Meta parameters
if viewPlots.metaParameterPlot
    plotMetaParameters(data, task)
end
%% Motor efficiency
if viewPlots.efficiencyMap
    plotEfficiencyMapWithOperatingPoints(data, task)
end
%% Generate data table in command window
tabulateResults(data, task)