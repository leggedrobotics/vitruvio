%% Plot values
close all;

% plotMotionData(EE, relativeMotionHipEE, meanCyclicMotionHipEE, IF_hip, tLiftoff, tTouchdown, samplingStart, samplingEnd)


%% comet plots of end effector motion wrt hip attachment points

% 
% figure()
% hold on
% title('Relative motion of end effector with respect to hip attachment point');
% subplot(2,2,1)
% title('Left front end effector');
% ax=axes;
% set(ax,'xlim',[x_min x_max], 'ylim',[z_min z_max]);
% % hold (ax)
% comet(ax,foot_LF_HipEE.position(:,1), foot_LF_HipEE.position(:,3));
% 
% subplot(2,2,2)
% title('Right front end effector');
% xlabel('x position [m]');
% zlabel('z position [m]');
% ax = axes;
% comet(ax,foot_RF_HipEE.position(:,1), foot_RF_HipEE.position(:,3));
% 
% subplot(2,2,3)
% title('Left hind end effector');
% xlabel('x position [m]');
% zlabel('z position [m]');
% ax = axes;
% comet(ax,foot_LH_HipEE.position(:,1), foot_LH_HipEE.position(:,3));
% 
% subplot(2,2,4)
% title('Right hind end effector');
% xlabel('x position [m]');
% zlabel('z position [m]');
% ax = axes;
% comet(ax,foot_RH_HipEE.position(:,1), foot_RH_HipEE.position(:,3));
% hold off


%% base position and velocity
figure();
subplot(5,2,1);
plot(t, base.position(:,1));
ylabel('x position [m]');
grid on
title('Center of mass position')
subplot(5,2,3);
plot(t, base.position(:,2));
ylabel('y position [m]');
grid on
subplot(5,2,5);
plot(t, base.position(:,3));
xlabel('time [s]');
ylabel('z position [m]');
grid on

subplot(5,2,2);
plot(t, base.velocity(:,1));
ylabel('x velocity [m/s]');
grid on
title('Center of mass velocity')
subplot(5,2,4);
plot(t, base.velocity(:,2));
ylabel('y velocity [m/s]');
grid on
subplot(5,2,6);
plot(t, base.velocity(:,3));
xlabel('time [s]');
ylabel('z velocity [m/s]');
grid on

subplot(5,2,[7 8 9 10])
plot(t, EE.LF.force(:,3), 'b', t, EE.LH.force(:,3), 'g', ...
     t, EE.RF.force(:,3), 'r', t, EE.RH.force(:,3), 'k')

xlabel('time [s]')
ylabel('force in z-direction [N]')
title('End effector force')
legend('LF','LH','RF','RH')
grid on


%% end effector forces
figure()
subplot(4,1,1)
plot(t, EE.LF.force(:,1), 'b', t, EE.LH.force(:,1), 'g', ...
     t, EE.RF.force(:,1), 'r', t, EE.RH.force(:,1), 'c')
title('End Effector forces')
ylabel('force in x-direction [N]')
legend('LF','LH','RF','RH')
grid on

subplot(4,1,2)
plot(t, EE.LF.force(:,2), 'b', t, EE.LH.force(:,2), 'g', ...
     t, EE.RF.force(:,2), 'r', t, EE.RH.force(:,2), 'c')
ylabel('force in y-direction [N]')
legend('LF','LH','RF','RH')
grid on

subplot(4,1,3)
plot(t, EE.LF.force(:,3), 'b', t, EE.LH.force(:,3), 'g', ...
     t, EE.RF.force(:,3), 'r', t, EE.RH.force(:,3), 'c')
ylabel('force in z-direction [N]')
legend('LF','LH','RF','RH')
grid on

subplot(4,1,4)
plot(t, EE.LF.force(:,4), 'b', t, EE.LH.force(:,4), 'g', ...
     t, EE.RF.force(:,4), 'r', t, EE.RH.force(:,4), 'c')
xlabel('time [s]')
ylabel('magnitude of force [N]')
legend('LF','LH','RF','RH')
grid on
 
%% plots of hip attachment points, end effectors z position and z forces

figure()
% hip attachment points and end effector z position
subplot(2,1,1)
plot(t, EE.LF.position(:,3), 'b', t, EE.LH.position(:,3), 'g', ...
     t, EE.RF.position(:,3), 'r', t, EE.RH.position(:,3), 'c', ... 
     t, IF_hip.LF.position(:,3), 'b', t, IF_hip.LH.position(:,3), 'g', ...
     t, IF_hip.RF.position(:,3), 'r', t, IF_hip.RH.position(:,3), 'c')
 
title('Hip attachment point and effector z position');
xlabel('time [s]')
ylabel('position in z direction [m]')
legend('LF','LH','RF','RH')
 
% end effector z forces
subplot(2,1,2)
plot(t, EE.LF.force(:,3), 'b', t, EE.LH.force(:,3), 'g', ...
     t, EE.RF.force(:,3), 'r', t, EE.RH.force(:,3), 'c');
title('End effector z forces');
xlabel('time [s]')
ylabel('End effector forces in z direction [m]')

%% relative motion of EE with respect to fixed hip

xMin = min([min(relativeMotionHipEE.LF.position(:,1)) min(relativeMotionHipEE.LH.position(:,1)), ...
            min(relativeMotionHipEE.RF.position(:,1)), min(relativeMotionHipEE.RH.position(:,1))]);
        
xMax = max([max(relativeMotionHipEE.LF.position(:,1)) max(relativeMotionHipEE.LH.position(:,1)), ...
             max(relativeMotionHipEE.RF.position(:,1)),max(relativeMotionHipEE.RH.position(:,1))]);       
         
zMin = min([min(relativeMotionHipEE.LF.position(:,3)) min(relativeMotionHipEE.LH.position(:,3)), ...
            min(relativeMotionHipEE.RF.position(:,3)), min(relativeMotionHipEE.RH.position(:,3))]);
        
zMax = max([max(relativeMotionHipEE.LF.position(:,3)) max(relativeMotionHipEE.LH.position(:,3)), ...
             max(relativeMotionHipEE.RF.position(:,3)),max(relativeMotionHipEE.RH.position(:,3))]);       
        
                 
axisMin = min(xMin,zMin);
axisMax = max(xMax, zMax);

figure(4)
title('Relative motion of end effectors with respect to hip attachment points')

subplot(1,2,1)
plot3(relativeMotionHipEE.LF.position(:,1),relativeMotionHipEE.LF.position(:,2), relativeMotionHipEE.LF.position(:,3), 'b',  ...
     relativeMotionHipEE.RF.position(:,1),relativeMotionHipEE.RF.position(:,2), relativeMotionHipEE.RF.position(:,3), 'g')
grid on
title('Front end effector motion')
legend('LF','RF')
xlim([xMin, xMax])
% zlim([zMin, zMax])
axis equal
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')

subplot(1,2,2)
plot3(relativeMotionHipEE.LH.position(:,1), relativeMotionHipEE.LH.position(:,2), relativeMotionHipEE.LH.position(:,3), 'r', ...
     relativeMotionHipEE.RH.position(:,1), relativeMotionHipEE.LH.position(:,2),relativeMotionHipEE.RH.position(:,3), 'c')
grid on
title('Hind end effector motion')
legend('LH','RH')
xlim([xMin, xMax])
% zlim([zMin, zMax])
axis equal
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')

%% Plot mean x vs z position of EE in cyclic motion and z forces 

figure(5)
title('End effector motion with all gait cycles')

% LF
subplot(4,2,1)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LF.position(:,1,i), cyclicMotionHipEE.LF.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.position(:,3),'r', 'LineWidth',3)
% plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LF')

hold off
% LH
subplot(4,2,2)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LH.position(:,1,i), cyclicMotionHipEE.LH.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.LH.position(:,1),meanCyclicMotionHipEE.LH.position(:,3),'r', 'LineWidth',3)
% plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LH')

hold off

% RF
subplot(4,2,3)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RF.position(:,1,i), cyclicMotionHipEE.RF.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.position(:,3),'r', 'LineWidth',3)
% plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RF')

hold off

% RH
subplot(4,2,4)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RH.position(:,1,i), cyclicMotionHipEE.RH.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.position(:,3),'r', 'LineWidth',3)
% plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RH')

% add end effector z motion with rectangle showing sampled range of data
subplot(4,2,[5 6])
plot(t, EE.LF.position(:,3), 'b', t, EE.LH.position(:,3), 'g', ...
     t, EE.RF.position(:,3), 'r', t, EE.RH.position(:,3), 'c')
 legend('LF','LH','RF','RH')
 ylabel('End effector z position [m]')
 title(['Data sampled from steps ' num2str(samplingStart), ' to ' num2str(samplingEnd+1)])

 
                       % coordinate of bottom left point [x,y]
                       % width, height
 rectangle('Position',[min(tLiftoff(samplingStart,:)) EE.LF.position(1,3)-0.1 ,...
                       max(tLiftoff(samplingEnd,:))-min(tLiftoff(samplingStart,:))  max(EE.LF.position(:,3))+0.2],...
            'LineWidth', 2)
hold off

subplot(4,2,[7 8])
plot(t, EE.LF.force(:,3), 'b', t, EE.LH.force(:,3), 'g', ...
     t, EE.RF.force(:,3), 'r', t, EE.RH.force(:,3), 'c')
 legend('LF','LH','RF','RH')
 xlabel('time [s]')
 ylabel('End effector z forces [N]')
                 
 rectangle('Position',[min(tLiftoff(samplingStart,:)),-100 ,...
                       max(tLiftoff(samplingEnd,:))-min(tLiftoff(samplingStart,:))  max([max(EE.LF.force(:,3)), max(EE.LH.force(:,3)), ...
                       max(EE.RF.force(:,3)), max(EE.RH.force(:,3))])+150], 'LineWidth', 2)

hold off

%% Plot mean x vs z position with reachable positions

%LF
figure()

subplot(2,2,1)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LF.position(:,1,i), cyclicMotionHipEE.LF.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.position(:,3),'r', 'LineWidth',3)
plot(reachablePositions.LF(:,1),reachablePositions.LF(:,2))
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LF')

hold off
% LH
subplot(2,2,2)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LH.position(:,1,i), cyclicMotionHipEE.LH.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.LH.position(:,1),meanCyclicMotionHipEE.LH.position(:,3),'r', 'LineWidth',3)
plot(reachablePositions.LH(:,1),reachablePositions.LH(:,2))
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LH')

hold off

% RF
subplot(2,2,3)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RF.position(:,1,i), cyclicMotionHipEE.RF.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.position(:,3),'r', 'LineWidth',3)
plot(reachablePositions.LF(:,1),reachablePositions.LF(:,2))
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RF')

hold off

% RH
subplot(2,2,4)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RH.position(:,1,i), cyclicMotionHipEE.RH.position(:,3,i), 'bo')
end
plot(meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.position(:,3),'r', 'LineWidth',3)
plot(reachablePositions.LH(:,1),reachablePositions.LH(:,2))
plot(0,0,'o')
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RH')

hold off

%% mean forces

figure()
% % x forces for all EEs
% subplot(1,3,1)
% plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.force(:,1), 'b', ...
%      meanCyclicMotionHipEE.LH.position(:,1), meanCyclicMotionHipEE.LH.force(:,1), 'g',...
%      meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.force(:,1), 'r',...
%      meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.force(:,1), 'c')
% 
% title('Average of x direction forces for sampled data')
% legend('LF','LH','RF','RH')
% ylabel('force [N]')
% grid on
% 
% % y forces
% subplot(1,3,2)
% plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.force(:,2), 'b', ...
%      meanCyclicMotionHipEE.LH.position(:,1), meanCyclicMotionHipEE.LH.force(:,2), 'g', ...
%      meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.force(:,2), 'r', ...
%      meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.force(:,2), 'c')  
%  
% title('Average of y direction forces for sampled data')
% legend('LF','LH','RF','RH')
% xlabel('Offset in x direction from end effector to hip attachment point [m]')
% grid on

% Z forces
% subplot(1,3,3)
plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.force(:,3), 'b', ...
     meanCyclicMotionHipEE.LH.position(:,1), meanCyclicMotionHipEE.LH.force(:,3), 'g', ...
     meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.force(:,3), 'r', ...
     meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.force(:,3), 'c')  
 
title('Average of z direction forces for sampled data')
legend('LF','LH','RF','RH')
xlabel('Offset in x direction from end effector to hip attachment point [m]')
xlabel('Force [N]')
grid on


% % zforces
% subplot(1,4,4)
% plot(meanCyclicMotionHipEE.LF.position(:,1), meanCyclicMotionHipEE.LF.force(:,2), 'b', ...
%      meanCyclicMotionHipEE.LH.position(:,1), meanCyclicMotionHipEE.LH.force(:,2), 'g', ...
%      meanCyclicMotionHipEE.RF.position(:,1), meanCyclicMotionHipEE.RF.force(:,2), 'r', ...
%      meanCyclicMotionHipEE.RH.position(:,1), meanCyclicMotionHipEE.RH.force(:,2), 'c')  
%  
% title('Average of z direction forces for sampled data')
% legend('LF','LH','RF','RH')
% xlabel('Offset in x direction from end effector to hip attachment point [m]')
% grid on
