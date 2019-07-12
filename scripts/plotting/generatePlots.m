function [] = generatePlots(viewPlots, data, task, optimizeLeg)

% viewPlots.optimization.optimizedLegPlot  = true; % master level switch to disable all the optimization plots
% viewPlots.optimization.jointDataPlot     = false;
% viewPlots.optimization.metaParameterPlot = false;
% viewPlots.optimization.efficiencyMap     = false;
% viewPlots.efficiencyMap      = false;

% Create title page for the pdf
classSelection = data.(task).basicProperties.classSelection;
figure()
set(gcf,'color','w')
text(0.5,0.5,['Results of vitruvio simulation for ', classSelection, ' ', task], 'FontSize', 18,'HorizontalAlignment', 'center');
set(gca, 'visible', 'off', 'xtick', []);
export_fig results.pdf -nocrop -append

if viewPlots.motionData
    plotMotionData(data, task);
end

if viewPlots.rangeOfMotionPlots
    plotRangeOfMotion(data, task);
end

if viewPlots.optimization.jointDataPlot
    % Can optionally plot a second data set. Can select 'linePlot' or
    % 'scatterPlot'. If the leg is optimized, the optimized result is
    % plotted against the nominal result.
                            % data set 1     data set2   optimized legs 
    plotJointDataForAllLegs(data, task, [], [], optimizeLeg, 'linePlot')
end

if viewPlots.optimization.metaParameterPlot
    plotMetaParameters(data, task)
end

if viewPlots.efficiencyMap
    plotEfficiencyMapWithOperatingPoints(data, task)
end

tabulateResults(data, task)