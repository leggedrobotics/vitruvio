function [] = plotMetaParameters(data, task)

t                    = data.(task).time;
base                 = data.(task).base; % base motion for each leg during its cycle
base.fullTrajectory  = data.(task).fullTrajectory.base;
force.fullTrajectory = data.(task).fullTrajectory.force;
legCount             = data.(task).basicProperties.legCount;
linkCount            = data.(task).basicProperties.linkCount;
EEnames              = data.(task).basicProperties.EEnames;
removalRatioStart    = data.(task).basicProperties.trajectory.removalRatioStart;
removalRatioEnd      = data.(task).basicProperties.trajectory.removalRatioEnd;
fontSize = 14; 
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
    valueLabels = string(data.(task).(EEselection).linkLengths);
    if data.(task).basicProperties.optimizedLegs.(EEselection)
        
        valueLabelsOpt = string(data.(task).(EEselection).linkLengthsOpt);
        
        figureName = 'Optimized link lengths' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10)
        set(gcf,'color','w')
        ax1 = subplot(1,2,1);
        pie(ax1, 100*data.(task).(EEselection).linkLengths, valueLabels) % multiply values by 100 to avoid having a partial pie plot
        title(ax1, 'Link lengths, nominal leg design [m]', 'FontSize', fontSize);
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
        ax2 = subplot(1,2,2);
        pie(ax2, 100*data.(task).(EEselection).linkLengthsOpt, valueLabelsOpt)
        title(ax2, 'Link lengths, optimized leg design [m]', 'FontSize', fontSize);
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        %% peak joint torque comparison of actuators
        if linkCount == 2
            legendLabels = {'HAA','HFE','KFE'};
        elseif linkCount == 3
            legendLabels = {'HAA','HFE','KFE', 'AFE'};
        elseif linkCount == 4
            legendLabels = {'HAA','HFE','KFE', 'AFE', 'DFE'};
        end

        valueLabels    = string(round(data.(task).metaParameters.jointTorqueMax.(EEselection)));
        valueLabelsOpt = string(round(data.(task).metaParameters.jointTorqueMaxOpt.(EEselection)));
        
        figureName = 'Peak joint torques' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10)
        set(gcf,'color','w')        
        ax1 = subplot(1,2,1);
        pie(ax1, data.(task).metaParameters.jointTorqueMax.(EEselection), valueLabels)
        title(ax1, 'Peak joint torques, nominal leg design [Nm]', 'FontSize', fontSize);        
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        ax2 = subplot(1,2,2);
        pie(ax2, data.(task).metaParameters.jointTorqueMaxOpt.(EEselection), valueLabelsOpt)
        title(ax2, 'Peak joint torques, optimized leg design [Nm]', 'FontSize', fontSize);        
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        %% energy consumption comparison of actuators
        
        valueLabelsMechEnergy    = string(data.(task).metaParameters.mechEnergyPerCycle.(EEselection));
        valueLabelsMechEnergyOpt = string(data.(task).metaParameters.mechEnergyPerCycleOpt.(EEselection));
        valueLabelsElecEnergy    = string(data.(task).metaParameters.elecEnergyPerCycle.(EEselection));
        valueLabelsElecEnergyOpt = string(data.(task).metaParameters.elecEnergyPerCycleOpt.(EEselection));
       
        figureName = 'Energy demand' + " " + EEselection;
        figure('name', figureName, 'DefaultAxesFontSize', 10)
        set(gcf,'color','w')        

        % Mechanical energy demand
        ax1 = subplot(2,2,1);
        pie(ax1,  data.(task).metaParameters.mechEnergyPerCycle.(EEselection), valueLabelsMechEnergy)
        title(ax1, 'Mechanical energy demand, nominal leg design [J/cycle]', 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        ax2 = subplot(2,2,2);
        pie(ax2, data.(task).metaParameters.mechEnergyPerCycleOpt.(EEselection), valueLabelsMechEnergyOpt)
        title(ax2, 'Mechanical energy demand, optimized leg design [J/cycle]', 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        % Electrical energy demand
        ax3 = subplot(2,2,3);
        pie(ax3,  data.(task).metaParameters.elecEnergyPerCycle.(EEselection), valueLabelsElecEnergy)
        title(ax3, 'Electrical energy demand, nominal leg design [J/cycle]', 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')

        ax4 = subplot(2,2,4);
        pie(ax4, data.(task).metaParameters.elecEnergyPerCycleOpt.(EEselection), valueLabelsElecEnergyOpt)
        title(ax4, 'Electrical energy demand, optimized leg design [J/cycle]', 'FontSize', fontSize);                
        legend(legendLabels,'Location','southoutside','Orientation','horizontal')
    end
end