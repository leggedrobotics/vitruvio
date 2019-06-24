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

figure()
ax1 = subplot(1,2,1);
pie(ax1, Leg.(EEselection).linkLengths, valueLabels)
title(ax1,'Link lengths of initial leg design [m]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')
ax2 = subplot(1,2,2);
pie(ax2, Leg.(EEselection).linkLengthsOpt, valueLabelsOpt)
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

figure()
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

valueLabels = string(Leg.metaParameters.energyPerCycle.(EEselection));
valueLabelsOpt = string(Leg.metaParameters.energyPerCycleOpt.(EEselection));

figure()
ax1 = subplot(1,2,1);
pie(ax1,  Leg.metaParameters.energyPerCycle.(EEselection), valueLabels)
title(ax1,'Energy demand of initial leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')
ax2 = subplot(1,2,2);
pie(ax2, Leg.metaParameters.energyPerCycleOpt.(EEselection), valueLabelsOpt)
title(ax2,'Energy demand of optimized leg design [J/cycle]');
legend(legendLabels,'Location','southoutside','Orientation','horizontal')