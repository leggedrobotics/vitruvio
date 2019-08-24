clc
clear X
warning('off','all')

% compare measured and simulated results
selectDataFile = 'ANYmalBearFastTrotMeasured.mat';
ANYmalMeasuredData  = load(selectDataFile);
savePDF = false;
lineWidth = 2;
% Simulated data
robot = 'ANYmalBear'; 
task  = 'trotSwing5';

% Set axis limits
qAxisLimitsFastTrot = [-0.6, 0.6];
qdotAxisLimitsFastTrot = [-10, 10];
torqueAxisLimitsFastTrot = [-60, 60];
powerAxisLimitsFastTrot = [-80, 80];
energyAxisLimitsFastTrot = [0, 25];

qAxisLimitsSlowTrot = [-0.6, 0.6];
qdotAxisLimitsSlowTrot = [-10, 10];
torqueAxisLimitsSlowTrot = [-60, 60];
powerAxisLimitsSlowTrot = [-40, 40];
energyAxisLimitsSlowTrot = [0, 18];

if strcmp(selectDataFile, 'ANYmalBearSlowTrotMeasured.mat')
    qAxisLimits      = qAxisLimitsSlowTrot;
    qdotAxisLimits   = qdotAxisLimitsSlowTrot;
    torqueAxisLimits = torqueAxisLimitsSlowTrot;
    powerAxisLimits  = powerAxisLimitsSlowTrot;
    energyAxisLimits = energyAxisLimitsSlowTrot;
else
    qAxisLimits      = qAxisLimitsFastTrot;
    qdotAxisLimits   = qdotAxisLimitsFastTrot;
    torqueAxisLimits = torqueAxisLimitsFastTrot;
    powerAxisLimits  = powerAxisLimitsFastTrot;
    energyAxisLimits = energyAxisLimitsFastTrot;
end

legCount          = results.(robot).(task).basicProperties.legCount;
linkCount         = results.(robot).(task).basicProperties.linkCount;
EEnames           = results.(robot).(task).basicProperties.EEnames;
removalRatioStart = results.(robot).(task).basicProperties.trajectory.removalRatioStart;
removalRatioEnd   = results.(robot).(task).basicProperties.trajectory.removalRatioEnd;
startTimeIndexSimulatedData = round(length(results.(robot).(task).fullTrajectory.r.EEdes.LF)*removalRatioStart);
startTimeIndexSimulatedData(startTimeIndexSimulatedData<1) = 1;
endTimeIndexSimulatedData = startTimeIndexSimulatedData + length(results.(robot).(task).LF.q)-1;
dt = results.(robot).(task).time(2) - results.(robot).(task).time(1);
optimizedLegs = results.(robot).(task).basicProperties.optimizedLegs;

loadSavedResultsOpt = 0;
if loadSavedResultsOpt
   % resultsOpt = load('resultsOpt.mat');
   for i = 1:legCount
       EEselection = EEnames(i,:);
       optimizedLegs.(EEselection) = true; 
       results.(robot).(task).(EEselection).jointTorqueOpt = resultsOpt.(EEselection).jointTorqueOpt;
       results.(robot).(task).(EEselection).mechEnergyOpt = resultsOpt.(EEselection).mechEnergyOpt;
   end
end
%% 
% Align data sets by shifting the measured data in time
timeShiftSlowTrot = -2.962;
startTimeSlowTrot = 1.5;
endTimeSlowTrot   = 5.5;

timeShiftFastTrot = -2.655;
startTimeFastTrot = 1.5;
endTimeFastTrot   = 3.5;

if strcmp(selectDataFile, 'ANYmalBearSlowTrotMeasured.mat') || strcmp(selectDataFile, 'ANYmalBearSlowTrot2Measured.mat')
    timeShift = timeShiftSlowTrot;
    startTime = startTimeSlowTrot; % plotted window limits
    endTime   = endTimeSlowTrot;
    endTimeMeasured = endTime - timeShift;
else
    timeShift = timeShiftFastTrot;
    startTime = startTimeFastTrot; % plotted window limits
    endTime   = endTimeFastTrot;
    endTimeMeasured = endTime - timeShift;    
end

% Compute power for measured data
ANYmalMeasuredData.joint.LF.meas.jointPower = ANYmalMeasuredData.joint.LF.meas.qdot .* ANYmalMeasuredData.joint.LF.meas.jointTorque;
ANYmalMeasuredData.joint.RF.meas.jointPower = ANYmalMeasuredData.joint.RF.meas.qdot .* ANYmalMeasuredData.joint.RF.meas.jointTorque;
ANYmalMeasuredData.joint.LH.meas.jointPower = ANYmalMeasuredData.joint.LH.meas.qdot .* ANYmalMeasuredData.joint.LH.meas.jointTorque;
ANYmalMeasuredData.joint.RH.meas.jointPower = ANYmalMeasuredData.joint.RH.meas.qdot .* ANYmalMeasuredData.joint.RH.meas.jointTorque;

% Compute energy for measured data
power.LF.meas = ANYmalMeasuredData.joint.LF.meas.jointPower;
power.RF.meas = ANYmalMeasuredData.joint.RF.meas.jointPower;
power.LH.meas = ANYmalMeasuredData.joint.LH.meas.jointPower;
power.RH.meas = ANYmalMeasuredData.joint.RH.meas.jointPower;

% Neglect recuperation
power.LF.meas(power.LF.meas<0) = 0;
power.RF.meas(power.RF.meas<0) = 0;
power.LH.meas(power.LH.meas<0) = 0;
power.RH.meas(power.RH.meas<0) = 0;

ANYmalMeasuredData.joint.LF.meas.jointEnergy = power.LF.meas(1:end-1,:) .* ANYmalMeasuredData.joint.dt;
ANYmalMeasuredData.joint.RF.meas.jointEnergy = power.RF.meas(1:end-1,:) .* ANYmalMeasuredData.joint.dt;
ANYmalMeasuredData.joint.LH.meas.jointEnergy = power.LH.meas(1:end-1,:) .* ANYmalMeasuredData.joint.dt;
ANYmalMeasuredData.joint.RH.meas.jointEnergy = power.RH.meas(1:end-1,:) .* ANYmalMeasuredData.joint.dt;

% Energy consumption measured only from a starting time after the accel
% phase
startTimeMeasured  = startTime - timeShift; % seconds
startIndexEnergy =  round(startTimeMeasured/mean(ANYmalMeasuredData.joint.dt));
dtMeasured = mean(ANYmalMeasuredData.joint.dt);

ANYmalMeasuredData.joint.LF.meas.jointEnergyCumulative = [zeros(length(ANYmalMeasuredData.joint.LF.meas.jointEnergy(1:startIndexEnergy-1,1)),3); cumsum(ANYmalMeasuredData.joint.LF.meas.jointEnergy(startIndexEnergy:end,:))];
ANYmalMeasuredData.joint.RF.meas.jointEnergyCumulative = [zeros(length(ANYmalMeasuredData.joint.RF.meas.jointEnergy(1:startIndexEnergy-1,1)),3); cumsum(ANYmalMeasuredData.joint.RF.meas.jointEnergy(startIndexEnergy:end,:))];
ANYmalMeasuredData.joint.LH.meas.jointEnergyCumulative = [zeros(length(ANYmalMeasuredData.joint.LH.meas.jointEnergy(1:startIndexEnergy-1,1)),3); cumsum(ANYmalMeasuredData.joint.LH.meas.jointEnergy(startIndexEnergy:end,:))];
ANYmalMeasuredData.joint.RH.meas.jointEnergyCumulative = [zeros(length(ANYmalMeasuredData.joint.RH.meas.jointEnergy(1:startIndexEnergy-1,1)),3); cumsum(ANYmalMeasuredData.joint.RH.meas.jointEnergy(startIndexEnergy:end,:))];

% Cumulative torque normalized by number of points in the plotted time
% frame 
ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative = [zeros(length(ANYmalMeasuredData.joint.LF.meas.jointTorque(1:startIndexEnergy-1,1)),3); cumsum(abs(ANYmalMeasuredData.joint.LF.meas.jointTorque(startIndexEnergy:end,:)))]/ ((endTime-startTime)/dtMeasured);
ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative = [zeros(length(ANYmalMeasuredData.joint.RF.meas.jointTorque(1:startIndexEnergy-1,1)),3); cumsum(abs(ANYmalMeasuredData.joint.RF.meas.jointTorque(startIndexEnergy:end,:)))]/ ((endTime-startTime)/dtMeasured);
ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative = [zeros(length(ANYmalMeasuredData.joint.LH.meas.jointTorque(1:startIndexEnergy-1,1)),3); cumsum(abs(ANYmalMeasuredData.joint.LH.meas.jointTorque(startIndexEnergy:end,:)))]/ ((endTime-startTime)/dtMeasured);
ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative = [zeros(length(ANYmalMeasuredData.joint.RH.meas.jointTorque(1:startIndexEnergy-1,1)),3); cumsum(abs(ANYmalMeasuredData.joint.RH.meas.jointTorque(startIndexEnergy:end,:)))]/ ((endTime-startTime)/dtMeasured);

results.(robot).(task).LF.jointTorqueCumulative = (cumsum(abs(results.(robot).(task).LF.jointTorque(:,2:3))) - sum(abs(results.(robot).(task).LF.jointTorque(1:round(startTime/dt),2:3)))) / ((endTime-startTime)/dt);
results.(robot).(task).RF.jointTorqueCumulative = (cumsum(abs(results.(robot).(task).RF.jointTorque(:,2:3))) - sum(abs(results.(robot).(task).RF.jointTorque(1:round(startTime/dt),2:3)))) / ((endTime-startTime)/dt);
results.(robot).(task).LH.jointTorqueCumulative = (cumsum(abs(results.(robot).(task).LH.jointTorque(:,2:3))) - sum(abs(results.(robot).(task).LH.jointTorque(1:round(startTime/dt),2:3)))) / ((endTime-startTime)/dt);
results.(robot).(task).RH.jointTorqueCumulative = (cumsum(abs(results.(robot).(task).RH.jointTorque(:,2:3))) - sum(abs(results.(robot).(task).RH.jointTorque(1:round(startTime/dt),2:3)))) / ((endTime-startTime)/dt);

for i = 1:legCount
    EEselection = EEnames(i,:);
    if optimizedLegs.(EEselection)
        results.(robot).(task).(EEselection).jointTorqueCumulativeOpt = (cumsum(abs(results.(robot).(task).(EEselection).jointTorqueOpt(:,2:3))) - sum(abs(results.(robot).(task).(EEselection).jointTorqueOpt(1:round(startTime/dt),2:3)))) / ((endTime-startTime)/dt);
    end
end

% Shift the measured data in time to align the plots
% fast trot
ANYmalMeasuredData.joint.time = ANYmalMeasuredData.joint.time + timeShift; 

% Start counting energy consumption of simulated at selected time
startingEnergyConsumptionTime = startTime; % seconds
for i = 1:3
    for j = 1: legCount
        EEselection = EEnames(j,:);    
        results.(robot).(task).(EEselection).mechEnergy(:,i) = results.(robot).(task).(EEselection).mechEnergy(:,i) - results.(robot).(task).(EEselection).mechEnergy(round(startTime/dt),i);
        if optimizedLegs.(EEselection)
            results.(robot).(task).(EEselection).mechEnergyOpt(:,i) = results.(robot).(task).(EEselection).mechEnergyOpt(:,i) - results.(robot).(task).(EEselection).mechEnergyOpt(round(startTime/dt),i);
        end
    end
end
% Shift angle data to set mean angle to zero. This helps align plots
for i = 1:3
    for j = 1: legCount
        EEselection = EEnames(j,:);   
        results.(robot).(task).(EEselection).q(:,i) = results.(robot).(task).(EEselection).q(:,i) - mean(results.(robot).(task).(EEselection).q(:,i));
        ANYmalMeasuredData.joint.(EEselection).meas.q(:,i) = ANYmalMeasuredData.joint.(EEselection).meas.q(:,i) - mean(ANYmalMeasuredData.joint.(EEselection).meas.q(:,i));
    end
end


    %% Create patches to represent swing/stance phase
    % initialize vectors for nominal limits to be empty
    qMax          = []; qMin          = [];
    qdotMax       = []; qdotMin       = [];
    torqueMax     = []; torqueMin     = [];
    powerMax      = []; powerMin      = [];
    mechEnergyMax = []; mechEnergyMin = [];
    elecEnergyMax = []; elecEnergyMin = [];
    
    for i = 1:legCount
        EEselection = EEnames(i,:); 

        % Return max and min values for nominal design
        maxq.(EEselection) = max(max(results.(robot).(task).(EEselection).actuatorq - mean(results.(robot).(task).(EEselection).actuatorq)));
        minq.(EEselection) = min(min(results.(robot).(task).(EEselection).actuatorq - mean(results.(robot).(task).(EEselection).actuatorq)));

        maxqdot.(EEselection) = max(max(results.(robot).(task).(EEselection).actuatorqdot));
        minqdot.(EEselection) = min(min(results.(robot).(task).(EEselection).actuatorqdot));

        maxTorque.(EEselection) = max(max(results.(robot).(task).(EEselection).actuatorTorque));
        minTorque.(EEselection) = min(min(results.(robot).(task).(EEselection).actuatorTorque));

        maxPower.(EEselection) = max(max(results.(robot).(task).(EEselection).jointPower));
        minPower.(EEselection) = min(min(results.(robot).(task).(EEselection).jointPower));

        maxMechEnergy.(EEselection) = max(max(results.(robot).(task).(EEselection).mechEnergy));
        minMechEnergy.(EEselection) = min(min(results.(robot).(task).(EEselection).mechEnergy));

        maxElecEnergy.(EEselection) = max(max(results.(robot).(task).(EEselection).elecEnergy));
        minElecEnergy.(EEselection) = min(min(results.(robot).(task).(EEselection).elecEnergy));

        % fill in the vectors for limits
        qMax          = [qMax, maxq.(EEselection)];                   qMin          = [qMin, minq.(EEselection)];
        qdotMax       = [qdotMax, maxqdot.(EEselection)];             qdotMin       = [qdotMin, minqdot.(EEselection)];
        torqueMax     = [torqueMax, maxTorque.(EEselection)];         torqueMin     = [torqueMin, minTorque.(EEselection)];
        powerMax      = [powerMax, maxPower.(EEselection)];           powerMin      = [powerMin, minPower.(EEselection)];
        mechEnergyMax = [mechEnergyMax, maxMechEnergy.(EEselection)]; mechEnergyMin = [mechEnergyMin, minMechEnergy.(EEselection)];
        elecEnergyMax = [elecEnergyMax, maxElecEnergy.(EEselection)]; elecEnergyMin = [elecEnergyMin, minElecEnergy.(EEselection)];
    end

    ylimit.actuatorq    = 2*[min(qMin), max(qMax)];
    ylimit.actuatorqdot = 2*[min(qdotMin), max(qdotMax)];
    ylimit.torque       = 2*[min(torqueMin), max(torqueMax)];
    ylimit.power        = 2*[min(powerMin), max(powerMax)];
    ylimit.mechEnergy   = 3*[min(mechEnergyMin), max(mechEnergyMax)];
    
    % Light blue patch behind stance phase, swing phase left white.
    patchColor = [0.5843 0.8157 0.9882]; % Light blue
    patchAlpha = 0.5; % Transparency
    
    Y_qTemp      = [ylimit.actuatorq(1); ylimit.actuatorq(2); ylimit.actuatorq(2); ylimit.actuatorq(1)];
    Y_qdotTemp   = [ylimit.actuatorqdot(1); ylimit.actuatorqdot(2); ylimit.actuatorqdot(2); ylimit.actuatorqdot(1)];
    Y_torqueTemp = [ylimit.torque(1); ylimit.torque(2); ylimit.torque(2); ylimit.torque(1)];
    Y_powerTemp  = [ylimit.power(1); ylimit.power(2); ylimit.power(2); ylimit.power(1)];
    Y_mechEnergyTemp  = [ylimit.mechEnergy(1); ylimit.mechEnergy(2); ylimit.mechEnergy(2); ylimit.mechEnergy(1)];

    %% Create patches based on stance of simulated data
    for i = 1:legCount
        EEselection = EEnames(i,:);
        tLiftoff.(EEselection)   = results.(robot).(task).(EEselection).tLiftoff;
        tTouchdown.(EEselection) = results.(robot).(task).(EEselection).tTouchdown;
        
        % Determine which phase the leg is in at the beginning of the sampled
        % motion
        if tLiftoff.(EEselection)(1) < tTouchdown.(EEselection)(1)
            startingPhase.(EEselection) = 'stance';
        else
            startingPhase.(EEselection) = 'swing';
        end
        % Determine phase at end of sampled motion
        if tLiftoff.(EEselection)(end) < tTouchdown.(EEselection)(end)
            endingPhase.(EEselection) = 'stance';
        else
            endingPhase.(EEselection) = 'swing';
        end    

        if strcmp(startingPhase.(EEselection), 'stance') % Leg starts in stance
            X.(EEselection)(1,1) = 0;
            X.(EEselection)(2,1) = 0;
            X.(EEselection)(3,1) = tLiftoff.(EEselection)(1);
            X.(EEselection)(4,1) = tLiftoff.(EEselection)(1);
            for j = 1:length(tLiftoff.(EEselection))-1 
                X.(EEselection)(1,j+1) = tTouchdown.(EEselection)(j);
                X.(EEselection)(2,j+1) = tTouchdown.(EEselection)(j);
                X.(EEselection)(3,j+1) = tLiftoff.(EEselection)(j+1);
                X.(EEselection)(4,j+1) = tLiftoff.(EEselection)(j+1);
            end
        else % Leg starts in swing
            for j = 1:min([length(tLiftoff.(EEselection)), length(tTouchdown.(EEselection))])
                X.(EEselection)(1,j) = tTouchdown.(EEselection)(j);
                X.(EEselection)(2,j) = tTouchdown.(EEselection)(j);
                X.(EEselection)(3,j) = tLiftoff.(EEselection)(j);
                X.(EEselection)(4,j) = tLiftoff.(EEselection)(j);
            end
        end

        if strcmp(endingPhase.(EEselection), 'stance')
            X.(EEselection)(1,j+2) = tTouchdown.(EEselection)(end);
            X.(EEselection)(2,j+2) = tTouchdown.(EEselection)(end);    
            X.(EEselection)(3,j+2) = length(results.(robot).(task).(EEselection).actuatorq); % Continue last patch until end of plotted data
            X.(EEselection)(4,j+2) = length(results.(robot).(task).(EEselection).actuatorq);
        end
        % Shift the points to the starting time of the sampled motion
        X.(EEselection) = X.(EEselection) + results.(robot).(task).time(startTimeIndexSimulatedData);
    
        Y_q.(EEselection)      = repmat(Y_qTemp, 1, length(X.(EEselection)(1,:)));
        Y_qdot.(EEselection)   = repmat(Y_qdotTemp, 1, length(X.(EEselection)(1,:)));
        Y_torque.(EEselection) = repmat(Y_torqueTemp, 1, length(X.(EEselection)(1,:)));
        Y_power.(EEselection)  = repmat(Y_powerTemp, 1, length(X.(EEselection)(1,:)));
        Y_mechEnergy.(EEselection)  = repmat(Y_mechEnergyTemp, 1, length(X.(EEselection)(1,:)));

    end

%% Create title page for pdf
figure('name', 'Title Slide', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
text(0.5,0.5,['Results of validation for ', robot, ' ', task], 'FontSize', 22,'HorizontalAlignment', 'center');
set(gca, 'visible', 'off', 'xtick', []);
if savePDF
    export_fig validation.pdf -nocrop -append
end

%% Motion plot
figure('name', 'Base Position', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
set(gca,'FontSize',20)
subplot(3,1,1)
plot(ANYmalMeasuredData.base.time, ANYmalMeasuredData.base.position(:,1), 'LineWidth', lineWidth);
title('Base position x');
grid on
subplot(3,1,2)
plot(ANYmalMeasuredData.base.time, ANYmalMeasuredData.base.position(:,2), 'LineWidth', lineWidth);
title('Base position y');
grid on
subplot(3,1,3)
plot(ANYmalMeasuredData.base.time, ANYmalMeasuredData.base.position(:,3), 'LineWidth', lineWidth);
title('Base position z');
grid on
% if savePDF
%     export_fig validation.pdf -nocrop -append
% end

lineColorMeas = 'k';
lineColorSim  = 'r';
lineColorOpt  = 'b';

%% Joint position
fontSize = 16;
figure('name', 'Joint Position', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
for i = 2:3
    subplot(2,4,4*(i-2)+1)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LF, Y_q.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LF.meas.q(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.q(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LF q_{HFE}');
    else
        title('LF q_{KFE}');
    end
    xlim([startTime, endTime]);
    grid on
    ylim(qAxisLimits)
    xlabel('time [s]')
    ylabel('q [rad]')
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+2)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RF, Y_q.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RF.meas.q(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RF.q(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RF q_{HFE}');
    else
        title('RF q_{KFE}');
    end
     xlim([startTime, endTime]);
    grid on
    ylim(qAxisLimits)
    xlabel('time [s]')
    ylabel('q [rad]')    
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+3)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LH, Y_q.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)              
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LH.meas.q(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LH.q(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LH q_{HFE}');
    else
        title('LH q_{KFE}');
    end  
    xlim([startTime, endTime]);
    grid on
    ylim(qAxisLimits)   
    xlabel('time [s]')
    ylabel('q [rad]')    
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+4)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RH, Y_q.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RH.meas.q(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RH.q(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RH q_{HFE}');
    else
        title('RH q_{KFE}');
    end 
    xlim([startTime, endTime]);
    grid on
    ylim(qAxisLimits)
    xlabel('time [s]')
    ylabel('q [rad]')    
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    hold off
end
if savePDF
    export_fig validation.pdf -nocrop -append
end
%% Joint velocity
figure('name', 'Joint Velocity', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
for i = 2:3
    subplot(2,4,4*(i-2)+1)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LF, 1.5*Y_qdot.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LF.meas.qdot(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.qdot(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LF qdot_{HFE}');
    else
        title('LF qdot_{KFE}');
    end      
    xlim([startTime, endTime]);
    grid on
    ylim(qdotAxisLimits)
    xlabel('time [s]')
    ylabel('qdot [rad/s]')    
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+2)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RF, 1.5*Y_qdot.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)               
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RF.meas.qdot(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RF.qdot(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RF qdot_{HFE}');
    else
        title('RF qdot_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(qdotAxisLimits)
    xlabel('time [s]')
    ylabel('qdot [rad/s]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+3)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LH, 1.5*Y_qdot.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)              
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LH.meas.qdot(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LH.qdot(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LH qdot_{HFE}');
    else
        title('LH qdot_{KFE}');
    end          
    xlim([startTime, endTime]);
    grid on
    ylim(qdotAxisLimits)    
    xlabel('time [s]')
    ylabel('qdot [rad/s]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+4)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RH, 1.5*Y_qdot.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)               
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RH.meas.qdot(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RH.qdot(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RH qdot_{HFE}');
    else
        title('RH qdot_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(qdotAxisLimits)    
    xlabel('time [s]')
    ylabel('qdot [rad/s]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    hold off
end
if savePDF
    export_fig validation.pdf -nocrop -append
end
%% Joint Torque
figure('name', 'Joint Torque', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
for i = 2:3
    subplot(2,4,4*(i-2)+1)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LF, Y_torque.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LF.meas.jointTorque(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.jointTorque(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LF Torque_{HFE}');
    else
        title('LF Torque_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(torqueAxisLimits)
    xlabel('time [s]')
    ylabel('Joint Torque [Nm]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+2)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RF, Y_torque.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                   
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RF.meas.jointTorque(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RF.jointTorque(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RF Torque_{HFE}');
    else
        title('RF Torque_{KFE}');
    end       
    xlim([startTime, endTime]);
    grid on
    ylim(torqueAxisLimits)   
    xlabel('time [s]')
    ylabel('Joint Torque [Nm]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+3)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LH, Y_torque.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LH.meas.jointTorque(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LH.jointTorque(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LH Torque_{HFE}');
    else
        title('LH Torque_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(torqueAxisLimits)    
    xlabel('time [s]')
    ylabel('Joint Torque [Nm]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+4)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RH, Y_torque.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                    
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RH.meas.jointTorque(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RH.jointTorque(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2 
        title('RH Torque_{HFE}');
    else
        title('RH Torque_{KFE}');
    end       
    xlim([startTime, endTime]);
    grid on
    ylim(torqueAxisLimits)    
    xlabel('time [s]')
    ylabel('Joint Torque [Nm]')       
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    hold off
end
if savePDF
    export_fig validation.pdf -nocrop -append
end
%% Joint Power
figure('name', 'Joint Power', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
for i = 2:3
    hold on
    subplot(2,4,4*(i-2)+1)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LF, Y_power.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                    
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LF.meas.jointPower(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.jointPower(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LF Pmech_{HFE}');
    else
        title('LF Pmech_{KFE}');
    end       
    xlim([startTime, endTime]);
    grid on
    ylim(powerAxisLimits)    
    if i == 2
        legend(p,'Measured','Simulated')
    end
    xlabel('time [s]')
    ylabel('Joint Power [W]')  
    
    subplot(2,4,4*(i-2)+2)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RF, Y_power.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RF.meas.jointPower(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RF.jointPower(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RF Pmech_{HFE}');
    else
        title('RF Pmech_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(powerAxisLimits)    
    xlabel('time [s]')
    ylabel('Joint Power [W]')  
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+3)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LH, Y_power.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.LH.meas.jointPower(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LH.jointPower(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LH Pmech_{HFE}');
    else
        title('LH Pmech_{KFE}');
    end        
    xlim([startTime, endTime]);
    grid on
    ylim(powerAxisLimits)    
    xlabel('time [s]')
    ylabel('Joint Power [W]')  
    if i == 2
        legend(p,'Measured','Simulated')
    end
    
    subplot(2,4,4*(i-2)+4)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RH, Y_power.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time, ANYmalMeasuredData.joint.RH.meas.jointPower(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RH.jointPower(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RH Pmech_{HFE}');
    else
        title('RH Pmech_{KFE}');
    end       
    xlim([startTime, endTime]);
    grid on
    ylim(powerAxisLimits)    
    xlabel('time [s]')
    ylabel('Joint Power [W]')  
    if i == 2
        legend(p,'Measured','Simulated')
    end
    hold off     
end
if savePDF
    export_fig validation.pdf -nocrop -append
end
%% Joint Energy
figure('name', 'Joint Energy', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
for i = 2:3
    hold on
    subplot(2,4,4*(i-2)+1)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LF, Y_mechEnergy.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.LF.meas.jointEnergyCumulative(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.mechEnergy(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LF Emech_{HFE}');
    else
        title('LF Emech_{KFE}');
    end        
    xlim([startTime, endTime]);
    ylim(energyAxisLimits);
    grid on
    xlabel('time [s]')
    ylabel('Mechanical Energy [J]')      
    if i == 2
        legend(p,'Measured','Simulated')
    end
   
    subplot(2,4,4*(i-2)+2)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RF, Y_mechEnergy.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                    
    p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.RF.meas.jointEnergyCumulative(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RF.mechEnergy(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('RF Emech_{HFE}');
    else
        title('RF Emech_{KFE}');
    end     
    xlim([startTime, endTime]);
    ylim(energyAxisLimits);    
    grid on
    xlabel('time [s]')
    ylabel('Mechanical Energy [J]')      
    if i == 2
        legend(p,'Measured','Simulated')
    end    
    
    subplot(2,4,4*(i-2)+3)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.LH, Y_mechEnergy.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.LH.meas.jointEnergyCumulative(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LH.mechEnergy(:,i), lineColorSim, 'LineWidth', lineWidth);
    if i == 2
        title('LH Emech_{HFE}');
    else
        title('LH Emech_{KFE}');
    end      
    xlim([startTime, endTime]);
    ylim(energyAxisLimits);    
    grid on
    xlabel('time [s]')
    ylabel('Mechanical Energy [J]')      
    if i == 2
        legend(p,'Measured','Simulated')
    end    
    
    subplot(2,4,4*(i-2)+4)
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.RH, Y_mechEnergy.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)                        
    p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.RH.meas.jointEnergyCumulative(:,i), lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).RH.mechEnergy(:,i), lineColorSim, 'LineWidth', lineWidth);
     if i == 2
        title('RH Emech_{HFE}');
    else
        title('RH Emech_{KFE}');
    end     
    xlim([startTime, endTime]);
    ylim(energyAxisLimits);    
    grid on
    xlabel('time [s]')
    ylabel('Mechanical Energy [J]')      
    if i == 2
        legend(p,'Measured','Simulated')
    end    
end
if savePDF
    export_fig validation.pdf -nocrop -append
end

%% Sum of energy over all joints
figure('name', 'Joint Energy Total', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
energyAxisLimits = 1.5*energyAxisLimits;
subplotOrder = [1, 2, 5, 6];
for i = 1:legCount
    EEselection = EEnames(i,:);
    subplot(2,4,subplotOrder(i))
    set(gca,'FontSize', fontSize)
    hold on
    patch(X.(EEselection), Y_mechEnergy.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)  
    if optimizedLegs.(EEselection)
        p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.(EEselection).meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.(EEselection).meas.jointEnergyCumulative(:,3), lineColorMeas, ...
             results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).(EEselection).mechEnergy(:,2)+results.(robot).(task).(EEselection).mechEnergy(:,3), lineColorSim, ...
             results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).(EEselection).mechEnergyOpt(:,2)+results.(robot).(task).(EEselection).mechEnergyOpt(:,3), lineColorOpt, 'LineWidth', lineWidth);
         legend(p,'Measured','Simulated', 'Optimized')
    else
        p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.(EEselection).meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.(EEselection).meas.jointEnergyCumulative(:,3), lineColorMeas, ...
             results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).(EEselection).mechEnergy(:,2)+results.(robot).(task).(EEselection).mechEnergy(:,3), lineColorSim, 'LineWidth', lineWidth);
        legend(p,'Measured','Simulated')
    end
    title([EEselection, ' ', 'E_{mechTotal}'])
    xlim([startTime, endTime]);
    ylim(energyAxisLimits);
    grid on
    xlabel('time [s]')
    ylabel('Normalized Mechanical Energy')      
end

subplot(2,4,[3,4,7,8])
set(gca,'FontSize', fontSize)
hold on

% Sum the energy for each leg. If the leg was optimized, use the optimized
% energy value.
results.(robot).(task).totalMechEnergy = [0, 0];
for i = 1:legCount
    EEselection = EEnames(i,:);
    if optimizedLegs.(EEselection)
        results.(robot).(task).totalMechEnergy = results.(robot).(task).totalMechEnergy + results.(robot).(task).(EEselection).mechEnergyOpt(:,2) + results.(robot).(task).(EEselection).mechEnergyOpt(:,3);
    else
        results.(robot).(task).totalMechEnergy = results.(robot).(task).totalMechEnergy + results.(robot).(task).(EEselection).mechEnergy(:,2) + results.(robot).(task).(EEselection).mechEnergy(:,3);
    end
end
 hold on  
set(gca,'FontSize', fontSize)
p = plot(ANYmalMeasuredData.joint.time(1:end-1), ANYmalMeasuredData.joint.LF.meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.LF.meas.jointEnergyCumulative(:,3) + ...
                                                 ANYmalMeasuredData.joint.RF.meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.RF.meas.jointEnergyCumulative(:,3) + ...
                                                 ANYmalMeasuredData.joint.LH.meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.LH.meas.jointEnergyCumulative(:,3) + ...
                                                 ANYmalMeasuredData.joint.RH.meas.jointEnergyCumulative(:,2) + ANYmalMeasuredData.joint.RH.meas.jointEnergyCumulative(:,3), lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).LF.mechEnergy(:,2) + results.(robot).(task).LF.mechEnergy(:,3) + ...
                                                                                         results.(robot).(task).RF.mechEnergy(:,2) + results.(robot).(task).RF.mechEnergy(:,3) + ...
                                                                                         results.(robot).(task).LH.mechEnergy(:,2) + results.(robot).(task).LH.mechEnergy(:,3) + ...
                                                                                         results.(robot).(task).RH.mechEnergy(:,2) + results.(robot).(task).RH.mechEnergy(:,3), lineColorSim, 'LineWidth', lineWidth);
                                                                                     
if optimizedLegs.LF || optimizedLegs.RF || optimizedLegs.LH || optimizedLegs.RH
   pOpt = plot(results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).totalMechEnergy, lineColorOpt);
    legend('Measured','Simulated', 'Optimized')
else 
    legend(p,'Measured','Simulated')
end
title('Sum Over All Legs E_{mechTotal}');
xlim([startTime, endTime]);
ylim(4*energyAxisLimits);    
grid on
xlabel('time [s]')
ylabel('Normalized Mechanical Energy')      
hold off
if savePDF
    export_fig validation.pdf -nocrop -append
end

%% Sum of torque over all joints
fontSize = 12;
figure('name', 'Joint Torque Total', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
set(gcf,'color','w')
endTorque.LF.meas = sum(ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),2) + ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),3));
endTorque.RF.meas = sum(ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),2) + ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),3));
endTorque.LH.meas = sum(ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),2) + ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),3));
endTorque.RH.meas = sum(ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),2) + ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(round(endTimeMeasured/dtMeasured),3));

subplot(2,4,1)
set(gca,'FontSize', fontSize)
hold on
patch(X.LF, Y_torque.LF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)   
if optimizedLegs.LF
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,3))/endTorque.LF.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LF.jointTorqueCumulative(:,1) + results.(robot).(task).LF.jointTorqueCumulative(:,2))/endTorque.LF.meas, lineColorSim, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LF.jointTorqueCumulativeOpt(:,1) + results.(robot).(task).LF.jointTorqueCumulativeOpt(:,2))/endTorque.LF.meas, lineColorOpt, 'LineWidth', lineWidth);
     legend(p,'Measured','Simulated', 'Optimized')    
else
p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,3))/endTorque.LF.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LF.jointTorqueCumulative(:,1) + results.(robot).(task).LF.jointTorqueCumulative(:,2))/endTorque.LF.meas, lineColorSim, 'LineWidth', lineWidth);
legend(p,'Measured','Simulated')    
end
title('LF Normalized Cumulative Joint Torque');

xlim([startTime, endTime]);
ylim([0, 1.2]);    
grid on
xlabel('time [s]')
ylabel('Normalized Joint Torque') 

subplot(2,4,2)
set(gca,'FontSize', fontSize)
hold on
patch(X.RF, Y_torque.RF, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)     
if optimizedLegs.RF
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,3))/endTorque.RF.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RF.jointTorqueCumulative(:,1) + results.(robot).(task).RF.jointTorqueCumulative(:,2))/endTorque.RF.meas, lineColorSim, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RF.jointTorqueCumulativeOpt(:,1) + results.(robot).(task).RF.jointTorqueCumulativeOpt(:,2))/endTorque.RF.meas, lineColorOpt, 'LineWidth', lineWidth);
     legend(p,'Measured','Simulated', 'Optimized')    
else
p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,3))/endTorque.RF.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RF.jointTorqueCumulative(:,1) + results.(robot).(task).RF.jointTorqueCumulative(:,2))/endTorque.RF.meas, lineColorSim, 'LineWidth', lineWidth);
legend(p,'Measured','Simulated')    
end
title('RF Normalized Cumulative Joint Torque');
xlim([startTime, endTime]);
ylim([0, 1.2]);    
grid on
xlabel('time [s]')
ylabel('Normalized Joint Torque') 

subplot(2,4,5)
set(gca,'FontSize', fontSize)
hold on
patch(X.LH, Y_torque.LH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)     
if optimizedLegs.LH
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,3))/endTorque.LH.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LH.jointTorqueCumulative(:,1) + results.(robot).(task).LH.jointTorqueCumulative(:,2))/endTorque.LH.meas, lineColorSim, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LH.jointTorqueCumulativeOpt(:,1) + results.(robot).(task).LH.jointTorqueCumulativeOpt(:,2))/endTorque.LH.meas, lineColorOpt, 'LineWidth', lineWidth);
     legend(p,'Measured','Simulated', 'Optimized')    
else
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,3))/endTorque.LH.meas, lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LH.jointTorqueCumulative(:,1) + results.(robot).(task).LH.jointTorqueCumulative(:,2))/endTorque.LH.meas, lineColorSim, 'LineWidth', lineWidth);
    legend(p,'Measured','Simulated')    
end
title('LH Normalized Cumulative Joint Torque');
xlim([startTime, endTime]);
ylim([0, 1.2]);    
grid on
xlabel('time [s]')
ylabel('Normalized Joint Torque') 

subplot(2,4,6)
set(gca,'FontSize', fontSize)
hold on
patch(X.RH, Y_torque.RH, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)  
if optimizedLegs.RH
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,3))/endTorque.RH.meas, lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RH.jointTorqueCumulative(:,1) + results.(robot).(task).RH.jointTorqueCumulative(:,2))/endTorque.RH.meas, lineColorSim, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RH.jointTorqueCumulativeOpt(:,1) + results.(robot).(task).RH.jointTorqueCumulativeOpt(:,2))/endTorque.RH.meas, lineColorOpt, 'LineWidth', lineWidth);
     legend(p,'Measured','Simulated', 'Optimized')    
else
    p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,3))/endTorque.RH.meas, lineColorMeas, ...
         results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).RH.jointTorqueCumulative(:,1) + results.(robot).(task).RH.jointTorqueCumulative(:,2))/endTorque.RH.meas, lineColorSim, 'LineWidth', lineWidth);
    legend(p,'Measured','Simulated')    
end 
title('RH Normalized Cumulative Joint Torque');
xlim([startTime, endTime]);
ylim([0, 1.2]);    
grid on
xlabel('time [s]')
ylabel('Normalized Joint Torque') 

subplot(2,4,[3,4,7,8])
set(gca,'FontSize', fontSize)
hold on
% Sum the torque for each leg. If the leg was optimized, use the optimized
% torque value.
results.(robot).(task).totalTorque = [0, 0];
for i = 1:legCount
    EEselection = EEnames(i,:);
    if optimizedLegs.(EEselection)
        results.(robot).(task).totalTorque = results.(robot).(task).totalTorque + results.(robot).(task).(EEselection).jointTorqueCumulativeOpt(:,1) + results.(robot).(task).(EEselection).jointTorqueCumulativeOpt(:,2);
    else
        results.(robot).(task).totalTorque = results.(robot).(task).totalTorque + results.(robot).(task).(EEselection).jointTorqueCumulative(:,1) + results.(robot).(task).(EEselection).jointTorqueCumulative(:,2);
    end
end
hold on    
endTorque.total.meas = endTorque.LF.meas + endTorque.RF.meas + endTorque.LH.meas + endTorque.RH.meas;
set(gca,'FontSize', fontSize)

p = plot(ANYmalMeasuredData.joint.time, (ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LF.meas.jointTorqueCumulative(:,3) + ...
                                         ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RF.meas.jointTorqueCumulative(:,3) + ...
                                         ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.LH.meas.jointTorqueCumulative(:,3) + ...
                                         ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,2) + ANYmalMeasuredData.joint.RH.meas.jointTorqueCumulative(:,3))/endTorque.total.meas , lineColorMeas, ...
     results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), (results.(robot).(task).LF.jointTorqueCumulative(:,1) + results.(robot).(task).LF.jointTorqueCumulative(:,2) + ...
                                                                                          results.(robot).(task).RF.jointTorqueCumulative(:,1) + results.(robot).(task).RF.jointTorqueCumulative(:,2) + ...
                                                                                          results.(robot).(task).LH.jointTorqueCumulative(:,1) + results.(robot).(task).LH.jointTorqueCumulative(:,2) + ...
                                                                                          results.(robot).(task).RH.jointTorqueCumulative(:,1) + results.(robot).(task).RH.jointTorqueCumulative(:,2))/endTorque.total.meas, lineColorSim, 'LineWidth', lineWidth);
if optimizedLegs.LF || optimizedLegs.RF || optimizedLegs.LH || optimizedLegs.RH
   pOpt = plot(results.(robot).(task).time(startTimeIndexSimulatedData:endTimeIndexSimulatedData), results.(robot).(task).totalTorque/endTorque.total.meas, lineColorOpt);
    legend('Measured','Simulated', 'Optimized')
else 
    legend(p,'Measured','Simulated')
end

title('Normalized Cumulative Joint Torque Over All Legs');
xlim([startTime, endTime]);
ylim([0, 1.2]);    
grid on
xlabel('time [s]')
ylabel('Normalized Joint Torque') 
if savePDF
    export_fig validation.pdf -nocrop -append
end