function [] = plotRangeOfMotion(quadruped, Leg, meanCyclicMotionHipEE)

%% get reachable positions
reachablePositions = getRangeofMotion(quadruped, Leg);

%% Plot mean x vs z position with reachable positions
%LF
figure(7)
subplot(2,2,1)
hold on
plot(reachablePositions.LF(:,1),reachablePositions.LF(:,2), 'color', [0.5843 0.8157 0.9882])
plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.position(:,3),'k', 'LineWidth',4)
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LF')
hold off

% LH
subplot(2,2,2)
hold on
plot(reachablePositions.LH(:,1),reachablePositions.LH(:,2), 'color', [0.5843 0.8157 0.9882])
plot(meanCyclicMotionHipEE.LH.position(:,1),meanCyclicMotionHipEE.LH.position(:,3),'k', 'LineWidth', 4)
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LH')
hold off

% RF
subplot(2,2,3)
hold on
plot(reachablePositions.LF(:,1),reachablePositions.LF(:,2), 'color', [0.5843 0.8157 0.9882])
plot(meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.position(:,3),'k', 'LineWidth',4)
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RF')
hold off

% RH
subplot(2,2,4)
hold on
plot(reachablePositions.LH(:,1),reachablePositions.LH(:,2), 'color', [0.5843 0.8157 0.9882])
plot(meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.position(:,3),'k', 'LineWidth', 4)
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RH')
hold off
