function [] = generatePlots(viewPlots, classSelection, task)

% viewPlots.optimization.optimizedLegPlot  = true; % master level switch to disable all the optimization plots
% viewPlots.optimization.jointDataPlot     = false;
% viewPlots.optimization.metaParameterPlot = false;
% viewPlots.optimization.efficiencyMap     = false;
% viewPlots.efficiencyMap      = false;

if viewPlots.motionData
    plotMotionData(classSelection, task);
end

if viewPlots.rangeOfMotionPlots
    plotRangeOfMotion(classSelection, task);
end

if viewPlots.optimization.jointDataPlot
    % Can optionally plot a second data set. Can select 'linePlot' or
    % 'scatterPlot'. If the leg is optimized, the optimized result is
    % plotted against the nominal result.
    plotJointDataForAllLegs(classSelection, task, [], [], classSelection.(task).basicProperties.optimizedLegs, 'linePlot')
end

if viewPlots.optimization.metaParameterPlot
    plotMetaParameters(classSelection, task)
end

if viewPlots.efficiencyMap
    plotEfficiencyMapWithOperatingPoints(classSelection, task)
end