clc
%close all
nom = load('nominal_walk_24,5cm_500gCW_pullingTM');
opt = load('optWalk_500gCW_pullTM_24,5cm');
time_nom = nom.time;
time_opt = opt.time;

timeAlignment = 3.7; % Align opt data with nom data in time
forceSensorTimeAlignment = 0.56;

% Multiply z force readings by -1 to be consistent with towr
nom.forceSensor(:,3) = -nom.forceSensor(:,3);
opt.forceSensor(:,3) = -opt.forceSensor(:,3);


xlimit     = [55 59.5];
ylimHFE    = [-2.5 2.5];
ylimKFE    = [-2.5 2.5];
ylimSensor = [-2 28];
patchColor = [0.5843 0.8157 0.9882]; % Light blue
patchAlpha = 0.3;

%% Create patches from force sensor to indicate swing and stance
patchX = [[40;40;55.33;55.33] [55.63;55.63;56.3;56.3] [56.6;56.6;57.28;57.28] [57.51;57.51;58.15;58.15] [58.39;58.39;58.59;58.59] [58.66;58.66;70;70]];
patchYtemp = [-100; 100; 100; -100];
patchY = repmat(patchYtemp, 1,length(patchX(1,:)));

%% Plot results
figure()
set(gcf,'color','w')

subplot(3,1,1)
hold on
patch(forceSensorTimeAlignment+patchX, patchY, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)
p(1) = plot(nom.time, nom.current_HFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.current_HFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Acutator Current HFE')
xlabel('Time [s]')
ylabel('Current [A]')
xlim(xlimit);
ylim(ylimHFE);
hold off

subplot(3,1,2)
hold on
patch(forceSensorTimeAlignment+patchX, patchY, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)
p(1) = plot(nom.time, nom.current_KFE, 'r', 'DisplayName', 'nom');
p(2) = plot(opt.time+timeAlignment, opt.current_KFE, 'b', 'DisplayName', 'opt');
legend([p(1) p(2)]);
grid on
title('Acutator Current KFE')
xlabel('Time [s]')
ylabel('Current [A]')
xlim(xlimit);
ylim(ylimKFE);
hold off

subplot(3,1,3)
hold on
patch(forceSensorTimeAlignment+patchX, patchY, patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)
p(3) = plot(nom.forceSensorTime+forceSensorTimeAlignment, nom.forceSensor(:,3), 'r', 'DisplayName', 'nom');
p(4) = plot(opt.forceSensorTime+timeAlignment+forceSensorTimeAlignment, opt.forceSensor(:,3), 'b', 'DisplayName', 'opt');
legend([p(3) p(4)]);
grid on
title('Force Sensor Readings')
xlabel('Time [s]')
ylabel('End effector force in z direction [N]')
xlim(xlimit);
ylim(ylimSensor);
hold off

