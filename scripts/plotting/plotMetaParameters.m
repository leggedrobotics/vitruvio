function [] = plotMetaParameters(Leg, linkCount, EEselection)
%% link lengths
if linkCount == 2
    legendLabels = {'Hip','Thigh','Shank'};
elseif linkCount == 3
    legendLabels = {'Hip','Thigh','Shank', 'Foot'};
elseif linkCount == 4
    legendLabels = {'Hip','Thigh','Shank', 'Foot'};
end

valueLabels = string(Leg.(EEselection).linkLengths);
valueLabelsOpt = string(Leg.(EEselection).linkLengthsOpt);

figure('name', 'Optimized link lengths', 'DefaultAxesFontSize', 10)
ax1 = subplot(1,2,1);
pie(ax1, 100*Leg.(EEselection).linkLengths, valueLabels) % multiply values by 100 to avoid having a partial pie plot
title(ax1,'Link lengths of initial leg design [m]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')
ax2 = subplot(1,2,2);
pie(ax2, 100*Leg.(EEselection).linkLengthsOpt, valueLabelsOpt)
title(ax2,'Link lengths of optimized leg design [m]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

%% peak joint torque comparison of actuators
if linkCount == 2
    legendLabels = {'HAA','HFE','KFE'};
elseif linkCount == 3
    legendLabels = {'HAA','HFE','KFE', 'AFE'};
elseif linkCount == 4
    legendLabels = {'HAA','HFE','KFE', 'AFE', 'DFE'};
end

valueLabels = string(round(Leg.metaParameters.jointTorqueMax.(EEselection)));
valueLabelsOpt = string(round(Leg.metaParameters.jointTorqueMaxOpt.(EEselection)));

figure('name', 'Peak joint torques', 'DefaultAxesFontSize', 10)
ax1 = subplot(1,2,1);
pie(ax1, Leg.metaParameters.jointTorqueMax.(EEselection), valueLabels)
title(ax1,'Peak joint torques of initial leg design [Nm]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

ax2 = subplot(1,2,2);
pie(ax2, Leg.metaParameters.jointTorqueMaxOpt.(EEselection), valueLabelsOpt)
title(ax2,'Peak joint torques of optimized leg design [Nm]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

%% energy consumption comparison of actuators
if linkCount == 2
    legendLabels = {'HAA','HFE','KFE'};
elseif linkCount == 3
    legendLabels = {'HAA','HFE','KFE', 'AFE'};
elseif linkCount == 4
    legendLabels = {'HAA','HFE','KFE', 'AFE', 'DFE'};
end

valueLabelsMechEnergy = string(Leg.metaParameters.mechEnergyPerCycle.(EEselection));
valueLabelsMechEnergyOpt = string(Leg.metaParameters.mechEnergyPerCycleOpt.(EEselection));
valueLabelsElecEnergy = string(Leg.metaParameters.elecEnergyPerCycle.(EEselection));
valueLabelsElecEnergyOpt = string(Leg.metaParameters.elecEnergyPerCycleOpt.(EEselection));

figure('name', 'Mechanical and Electrical Energy Demand', 'DefaultAxesFontSize', 10)

%% mechanical energy demand
ax1 = subplot(2,2,1);
pie(ax1,  Leg.metaParameters.mechEnergyPerCycle.(EEselection), valueLabelsMechEnergy)
title(ax1,'Mechanical energy demand of initial leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

ax2 = subplot(2,2,2);
pie(ax2, Leg.metaParameters.mechEnergyPerCycleOpt.(EEselection), valueLabelsMechEnergyOpt)
title(ax2,'Mechanical energy demand of optimized leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

%% electrical energy demand
ax3 = subplot(2,2,3);
pie(ax3,  Leg.metaParameters.elecEnergyPerCycle.(EEselection), valueLabelsElecEnergy)
title(ax3,'Electrical energy demand of initial leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')

ax4 = subplot(2,2,4);
pie(ax4, Leg.metaParameters.elecEnergyPerCycleOpt.(EEselection), valueLabelsElecEnergyOpt)
title(ax4,'Electrical energy demand of optimized leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')