function [] = plotMetaParameters(data, saveFiguresToPDF)

t                    = data.time;
base                 = data.base; % base motion for each leg during its cycle
base.fullTrajectory  = data.fullTrajectory.base;
force.fullTrajectory = data.fullTrajectory.force;
legCount             = data.basicProperties.legCount;
linkCount            = data.basicProperties.linkCount;
EEnames              = data.basicProperties.EEnames;
removalRatioStart    = data.basicProperties.trajectory.removalRatioStart;
removalRatioEnd      = data.basicProperties.trajectory.removalRatioEnd;
fontSize = 10; 

%% link lengths
if linkCount == 2
    legendLabels = {'Hip','Thigh','Shank'};
elseif linkCount == 3
    legendLabels = {'Hip','Thigh','Shank', 'Foot'};
elseif linkCount == 4
    legendLabels = {'Hip','Thigh','Shank', 'Foot'};
end

for i = 1:legCount
    EEselection = EEnames(i,:);
    valueLabels = string(data.(EEselection).linkLengths);
    if data.basicProperties.optimizedLegs.(EEselection)
        
        valueLabelsOpt = string(data.(EEselection).linkLengthsOpt);
        
        figureName = 'Optimized link lengths' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
        set(gcf,'color','w')
        ax1 = subplot(1,2,1);
        pie(ax1, 100*data.(EEselection).linkLengths, valueLabels) % multiply values by 100 to avoid having a partial pie plot
        title(ax1, ({'Link lengths [m]',['Nominal ', EEselection]}), 'FontSize', fontSize);
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart
        
        ax2 = subplot(1,2,2);
        pie(ax2, 100*data.(EEselection).linkLengthsOpt, valueLabelsOpt)
        title(ax2, ({'Link lengths [m]',['Optimized ', EEselection]}), 'FontSize', fontSize);
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart
        if saveFiguresToPDF
            export_fig results.pdf -nocrop -append
        end

        %% Peak joint torque comparison of actuators
        if linkCount == 2
            legendLabels = {'HAA','HFE','KFE'};
        elseif linkCount == 3
            legendLabels = {'HAA','HFE','KFE', 'AFE'};
        elseif linkCount == 4
            legendLabels = {'HAA','HFE','KFE', 'AFE', 'DFE'};
        end

        valueLabels    = string(round(data.metaParameters.jointTorqueMax.(EEselection)));
        valueLabelsOpt = string(round(data.metaParameters.jointTorqueMaxOpt.(EEselection)));
        
        figureName = 'Peak joint torques' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
        set(gcf,'color','w')        
        ax1 = subplot(1,2,1);
        pie(ax1, data.metaParameters.jointTorqueMax.(EEselection), valueLabels)
        title(ax1, ({'Peak joint torques [Nm]',['Nominal ', EEselection]}), 'FontSize', fontSize);        
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart

        ax2 = subplot(1,2,2);
        pie(ax2, data.metaParameters.jointTorqueMaxOpt.(EEselection), valueLabelsOpt)
        title(ax2, ({'Peak joint torques [Nm]',['Optimized ', EEselection]}), 'FontSize', fontSize);        
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart
        if saveFiguresToPDF
            export_fig results.pdf -nocrop -append
        end

        %% Energy consumption comparison of actuators
       
        valueLabelsMechEnergy    = string(round(data.metaParameters.mechEnergyPerCycle.(EEselection)));
        valueLabelsMechEnergyOpt = string(round(data.metaParameters.mechEnergyPerCycleOpt.(EEselection)));
        valueLabelsElecEnergy    = string(round(data.metaParameters.elecEnergyPerCycle.(EEselection)));
        valueLabelsElecEnergyOpt = string(round(data.metaParameters.elecEnergyPerCycleOpt.(EEselection)));
       
        % Mechanical energy demand        
        figureName = 'Mechanical energy' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
        set(gcf,'color','w')        

        ax1 = subplot(1,2,1);
        pie(ax1,  data.metaParameters.mechEnergyPerCycle.(EEselection), valueLabelsMechEnergy)
        title(ax1, ({'Mechanical energy [J/cycle]',['Nominal ', EEselection]}), 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart

        ax2 = subplot(1,2,2);
        pie(ax2, data.metaParameters.mechEnergyPerCycleOpt.(EEselection), valueLabelsMechEnergyOpt)
        title(ax2, ({'Mechanical energy [J/cycle]',['Optimized ', EEselection]}), 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart
        if saveFiguresToPDF
            export_fig results.pdf -nocrop -append
        end
 
        % Electrical energy demand
        figureName = 'Electrical demand' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
        set(gcf,'color','w')    

        ax1 = subplot(1,2,1);
        pie(ax1,  data.metaParameters.elecEnergyPerCycle.(EEselection), valueLabelsElecEnergy)
        title(ax1, ({'Electrical energy [J/cycle]',['Nominal ', EEselection]}), 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart

        ax2 = subplot(1,2,2);
        pie(ax2, data.metaParameters.elecEnergyPerCycleOpt.(EEselection), valueLabelsElecEnergyOpt)
        title(ax2, ({'Electrical energy [J/cycle]',['Optimized ', EEselection]}), 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        view([90 90])   % this is to rotate the chart 
        if saveFiguresToPDF
            export_fig results.pdf -nocrop -append        
        end
    end
end