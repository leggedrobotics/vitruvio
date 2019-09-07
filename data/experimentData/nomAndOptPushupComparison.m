clc
close all
plotOnePushup = true;

nom = load('nom_pushup_adjusting_foot_position_3.mat');
opt = load('opt_pushup_no_counterweight.mat');
sim = load('pushupSimulation.mat');

time_nom = nom.time;
time_opt = opt.time;
if plotOnePushup
    timeAlignment = 358.5; % Align opt data with nom data in time
    timeAlignmentSim = 391.0;
    forceSensorTimeAlignment = 0;
else
    timeAlignment = 358.4; % Align opt data with nom data in time
    timeAlignmentSim = 391.0;
    forceSensorTimeAlignment = 0;
end
%% Approximate torque from current (improve mapping later)
nom.torque_HFE = 1.036*nom.current_HFE; %1.254*nom.current_HFE;
nom.torque_KFE = 2.2857*nom.current_KFE;

opt.torque_HFE = 1.036*opt.current_HFE;
opt.torque_KFE = 2.2857*opt.current_KFE;

% Multiply z force readings by -1 to be consistent with towr
nom.forceSensor(:,3) = -nom.forceSensor(:,3);
opt.forceSensor(:,3) = -opt.forceSensor(:,3);

%% Get values for one pushup which are alligned and of equal length
nomCropped.time = nom.time(9200:9800);
nomCropped.current_HFE = nom.current_HFE(9200:9800);
nomCropped.current_KFE = nom.current_KFE(9200:9800);
nomCropped.torque_HFE = nom.torque_HFE(9200:9800);
nomCropped.torque_KFE = nom.torque_KFE(9200:9800);

optCropped.time = opt.time(3000:3600);
optCropped.current_HFE = opt.current_HFE(3000:3600);
optCropped.current_KFE = opt.current_KFE(3000:3600);
optCropped.torque_HFE = opt.torque_HFE(9200:9800);
optCropped.torque_KFE = opt.torque_KFE(9200:9800);

%% Save alligned values into array for box plot
allignedCurrentData = [nomCropped.current_HFE, optCropped.current_HFE, nomCropped.current_KFE, optCropped.current_KFE];

%% Plot the current and force readings
% adjusting foot position 1: time +247.2, [273 283] [-0.5 1.5]
if plotOnePushup
    xlimit     = [391 392.8]; %[370 420];
    ylimHFE    = [-0.1 4];
    ylimKFE    = [-0.1 4];
    ylimSensor = [0 18];
else
    xlimit     = [370 420];
    ylimHFE    = [-0.1 4];
    ylimKFE    = [-0.1 4];
    ylimSensor = [0 18];
end

figure()
set(gcf,'color','w')

subplot(2,2,1)
hold on
p(1) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorTorque(:,2), 'r', 'DisplayName', 'nom');
p(2) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorTorqueOpt(:,2), 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Simulated Acutator Torque HFE')
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim(xlimit);
ylim([0 4]);
hold off

subplot(2,2,2)
hold on
p(1) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorTorque(:,3), 'r', 'DisplayName', 'nom');
p(2) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorTorqueOpt(:,3), 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Simulated Acutator Torque KFE')
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim(xlimit);
ylim([0 4]);
hold off

subplot(2,2,3)
hold on
p(1) = plot(nom.time, nom.torque_HFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.torque_HFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Measured Acutator Torque HFE')
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim(xlimit);
ylim(ylimHFE);
hold off

subplot(2,2,4)
hold on
p(1) = plot(nom.time, nom.torque_KFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.torque_KFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Measured Acutator Torque KFE')
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim(xlimit);
ylim(ylimKFE);
hold off

% subplot(3,1,3)
% hold on
% p(3) = plot(nom.forceSensorTime+forceSensorTimeAlignment, nom.forceSensor(:,3), 'r', 'DisplayName', 'nom');
% p(4) = plot(opt.forceSensorTime+timeAlignment+forceSensorTimeAlignment, opt.forceSensor(:,3), 'b', 'DisplayName', 'opt');
% legend([p(3) p(4)]);
% grid on
% title('Force Sensor Readings')
% xlabel('Time [s]')
% ylabel('End effector force in z direction [N]')
% xlim(xlimit);
% ylim(ylimSensor);
% hold off

%% Plot angle and velocity readings
figure()
set(gcf,'color','w')

subplot(2,2,1)
hold on
p(1) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorqdot(:,2), 'r', 'DisplayName', 'nom');
p(2) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorqdotOpt(:,2), 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Simulated Actuator Velocity HFE')
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
xlim(xlimit);
hold off

subplot(2,2,2)
hold on
p(1) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorqdot(:,3), 'r', 'DisplayName', 'nom');
p(2) = plot(sim.time(1:end-3)+timeAlignmentSim, sim.RF.actuatorqdotOpt(:,3), 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Simulated Actuator Velocity KFE')
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
xlim(xlimit);
hold off

subplot(2,2,3)
hold on
p(1) = plot(nom.time, nom.velocity_HFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.velocity_HFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Measured Actuator Velocity HFE')
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
xlim(xlimit);
hold off
hold off

subplot(2,2,4)
hold on
p(1) = plot(nom.time, nom.velocity_KFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.velocity_KFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Measured Actuator Velocity KFE')
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
xlim(xlimit);
hold off
hold off