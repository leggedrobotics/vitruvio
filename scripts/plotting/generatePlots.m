function [] = generatePlots(viewPlots, data, task, optimizeLeg, saveFiguresToPDF)
%% Create title page for the pdf

classSelection = data.basicProperties.classSelection;
if saveFiguresToPDF
    figure('name', 'Title Slide', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    text(0.5,0.5,['Results of vitruvio simulation for ', classSelection, ' ', task], 'FontSize', 22,'HorizontalAlignment', 'center');
    set(gca, 'visible', 'off', 'xtick', []);
    export_fig results.pdf -nocrop -append
end

%% Motion plot
if viewPlots.motionData
    fprintf('Plotting motion data. \n')
    plotMotionData(data, saveFiguresToPDF);
end
%% Joint data
if viewPlots.jointDataPlot
    fprintf('Plotting actuator speed, torque, power and energy data. \n')    
    plotJointDataForAllLegs(viewPlots, data,[], optimizeLeg, saveFiguresToPDF) % Can optionally plot a second data set on the same axes
end
%% Motor efficiency
if viewPlots.efficiencyMap
    fprintf('Plotting motor efficiency map. \n')    
    plotEfficiencyMapWithOperatingPoints(data, saveFiguresToPDF)
end
%% Generate data table in command window
tabulateResults(data)