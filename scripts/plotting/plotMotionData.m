%% Plot trajectory data
% set(gcf,'color','w');

%% base position and velocity
figure(2)
subplot(3,2,1);
plot(t, base.position(:,1));
ylabel('x position [m]');
grid on
title('Center of mass position')
subplot(3,2,3);
plot(t, base.position(:,2));
ylabel('y position [m]');
grid on
subplot(3,2,5);
plot(t, base.position(:,3));
xlabel('time [s]');
ylabel('z position [m]');
grid on

subplot(3,2,2);
plot(t, base.velocity(:,1));
ylabel('x velocity [m/s]');
grid on
title('Center of mass velocity')
subplot(3,2,4);
plot(t, base.velocity(:,2));
ylabel('y velocity [m/s]');
grid on
subplot(3,2,6);
plot(t, base.velocity(:,3));
xlabel('time [s]');
ylabel('z velocity [m/s]');
grid on

%% end effector forces
figure(3)
% use time vector with interpolated values
subplot(3,1,1)
plot(Leg.time, EE.LF.force(1:end-1,1), 'b', Leg.time, EE.LH.force(1:end-1,1), 'g', ...
     Leg.time, EE.RF.force(1:end-1,1), 'r', Leg.time, EE.RH.force(1:end-1,1), 'c')
title('End Effector forces')
ylabel('force in x-direction [N]')
lgd = legend('LF','LH','RF','RH');
lgd.FontSize = 16;
grid on

subplot(3,1,2)
plot(Leg.time, EE.LF.force(1:end-1,2), 'b', Leg.time, EE.LH.force(1:end-1,2), 'g', ...
     Leg.time, EE.RF.force(1:end-1,2), 'r', Leg.time, EE.RH.force(1:end-1,2), 'c')
ylabel('force in y-direction [N]')
lgd = legend('LF','LH','RF','RH');
lgd.FontSize = 16;
grid on

subplot(3,1,3)
plot(Leg.time, EE.LF.force(1:end-1,3), 'b', Leg.time, EE.LH.force(1:end-1,3), 'g', ...
     Leg.time, EE.RF.force(1:end-1,3), 'r', Leg.time, EE.RH.force(1:end-1,3), 'c')
ylabel('force in z-direction [N]')
lgd = legend('LF','LH','RF','RH');
lgd.FontSize = 16;
grid on
 
%% relative motion of EE with respect to fixed hip displayed in 3D

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
lgd = legend('LH','RH');
lgd.FontSize = 16;
xlim([xMin, xMax])
% zlim([zMin, zMax])
axis equal
xlabel('x position [m]')
ylabel('y position [m]')
zlabel('z position [m]')

%% Plot mean x vs z position of EE in cyclic motion
figure(5)
title('End effector cyclic motion')
% LF
subplot(2,2,1)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LF.position(:,1,i), cyclicMotionHipEE.LF.position(:,3,i), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
end
plot(meanCyclicMotionHipEE.LF.position(1:meanTouchdownIndex.LF,1), meanCyclicMotionHipEE.LF.position(1:meanTouchdownIndex.LF,3),'k:', 'LineWidth',4)
plot(meanCyclicMotionHipEE.LF.position(meanTouchdownIndex.LF:end,1), meanCyclicMotionHipEE.LF.position(meanTouchdownIndex.LF:end,3), 'k', 'LineWidth',4)
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LF')
hold off

% LH
subplot(2,2,2)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.LH.position(:,1,i), cyclicMotionHipEE.LH.position(:,3,i), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
end
plot(meanCyclicMotionHipEE.LH.position(1:meanTouchdownIndex.LH,1), meanCyclicMotionHipEE.LH.position(1:meanTouchdownIndex.LH,3),'k:', 'LineWidth',4)
plot(meanCyclicMotionHipEE.LH.position(meanTouchdownIndex.LH:end,1), meanCyclicMotionHipEE.LH.position(meanTouchdownIndex.LH:end,3),'k', 'LineWidth',4)
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('LH')
hold off

% RF
subplot(2,2,3)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RF.position(:,1,i), cyclicMotionHipEE.RF.position(:,3,i), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
end
plot(meanCyclicMotionHipEE.RF.position(1:meanTouchdownIndex.RF,1), meanCyclicMotionHipEE.RF.position(1:meanTouchdownIndex.RF,3),'k:', 'LineWidth',4)
plot(meanCyclicMotionHipEE.RF.position(meanTouchdownIndex.RF:end,1), meanCyclicMotionHipEE.RF.position(meanTouchdownIndex.RF:end,3),'k', 'LineWidth',4)
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RF')
hold off

% RH
subplot(2,2,4)
for i = samplingStart:samplingEnd
    hold on
    plot(cyclicMotionHipEE.RH.position(:,1,i), cyclicMotionHipEE.RH.position(:,3,i), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
end
plot(meanCyclicMotionHipEE.RH.position(1:meanTouchdownIndex.RH,1), meanCyclicMotionHipEE.RH.position(1:meanTouchdownIndex.RH,3),'k:', 'LineWidth',4)
plot(meanCyclicMotionHipEE.RH.position(meanTouchdownIndex.RH:end,1), meanCyclicMotionHipEE.RH.position(meanTouchdownIndex.RH:end,3),'k', 'LineWidth',4)
axis equal
xlabel('x position [m]')
ylabel('z position [m]')
title('RH')

%% End effector z motion with rectangle showing sampled range of data
figure(6)
subplot(2,2,[1 2])
plot(t, EE.LF.position(:,3), 'b', t, EE.LH.position(:,3), 'g', ...
     t, EE.RF.position(:,3), 'r', t, EE.RH.position(:,3), 'c')
lgd = legend('LF','LH','RF','RH');
lgd.FontSize = 16;
 ylabel('End effector z position [m]')
 title(['Data sampled from steps ' num2str(samplingStart), ' to ' num2str(samplingEnd+1)])
 % draw rectangle on plot by specifying bottom left coordinate, width and
 % height
    rectanglePos.bottomLeft = [min([min(tLiftoff.LF(samplingStart)), ...
                                    min(tLiftoff.LH(samplingStart)), ...
                                    min(tLiftoff.RF(samplingStart)), ...
                                    min(tLiftoff.RH(samplingStart))]), ...
                                    EE.LF.position(1,3)-0.1];
    rectanglePos.width =  max([max(tLiftoff.LF(samplingEnd)), ...
                               max(tLiftoff.LH(samplingEnd)), ...
                               max(tLiftoff.RF(samplingEnd)), ...
                               max(tLiftoff.RH(samplingEnd))]) - rectanglePos.bottomLeft(1);                      
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, max(EE.LF.position(:,3))+0.2]); %, 'LineWidth', 2)             
hold off
clear rectanglePos

subplot(2,2,[3 4])
plot(Leg.time, EE.LF.force(1:end-1,3), 'b', Leg.time, EE.LH.force(1:end-1,3), 'g', ...
     Leg.time, EE.RF.force(1:end-1,3), 'r', Leg.time, EE.RH.force(1:end-1,3), 'c')
 lgd = legend('LF','LH','RF','RH');
 lgd.FontSize = 16;
 xlabel('time [s]')
 ylabel('End effector z forces [N]')
 % draw rectangle on plot by specifying bottom left coordinate, width and
 % height    
    rectanglePos.bottomLeft =   [min([min(tLiftoff.LF(samplingStart)), ...
                                    min(tLiftoff.LH(samplingStart)), ...
                                    min(tLiftoff.RF(samplingStart)), ...
                                    min(tLiftoff.RH(samplingStart))]), ...
                                    -100];
    rectanglePos.width =  max([max(tLiftoff.LF(samplingEnd)), ...
                               max(tLiftoff.LH(samplingEnd)), ...
                               max(tLiftoff.RF(samplingEnd)), ...
                               max(tLiftoff.RH(samplingEnd))]) - rectanglePos.bottomLeft(1);   

    rectanglePos.height =  max([max(EE.LF.force(:,3)), ...
                               max(EE.LH.force(:,3)), ...
                               max(EE.RF.force(:,3)), ...
                               max(EE.RH.force(:,3))] + 150);

    rectangle('Position',[rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 2)
    clear rectanglePos
hold off