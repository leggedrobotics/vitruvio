%% Plot trajectory data
function [] = plotMotionData(data, task)

t                    = data.(task).time;
base                 = data.(task).base; % base motion for each leg during its cycle
base.fullTrajectory  = data.(task).fullTrajectory.base;
force.fullTrajectory = data.(task).fullTrajectory.force;
legCount             = data.(task).basicProperties.legCount;
EEnames              = data.(task).basicProperties.EEnames;
removalRatioStart    = data.(task).basicProperties.trajectory.removalRatioStart;
removalRatioEnd      = data.(task).basicProperties.trajectory.removalRatioEnd;

%% Base position and velocity in inertial frame.
figure(2)
set(gcf,'color','w')
subplot(3,2,1);
plot(t, base.fullTrajectory.position(:,1));
ylabel('x position [m]');
grid on
title('Center of mass position')

subplot(3,2,3);
plot(t, base.fullTrajectory.position(:,2));
ylabel('y position [m]');
grid on

subplot(3,2,5);
plot(t, base.fullTrajectory.position(:,3));
xlabel('time [s]');
ylabel('z position [m]');
grid on

subplot(3,2,2);
plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,1));
ylabel('x velocity [m/s]');
grid on
title('Center of mass velocity')

subplot(3,2,4);
plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,2));
ylabel('y velocity [m/s]');
grid on

subplot(3,2,6);
plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,3));
xlabel('time [s]');
ylabel('z velocity [m/s]');
grid on

%% End effector forces.
figure(3)
set(gcf,'color','w')
lineColor = {'b', 'g', 'r', 'c'};
for i = 1:legCount
    subplot(3,1,1)
    hold on
    EEselection = EEnames(i,:);
    p(i) = plot(t, force.fullTrajectory.(EEselection)(1:length(t),1), lineColor{i}, 'DisplayName', EEselection);
    title('End Effector forces')
    xlabel('time [s]')    
    ylabel('force in x-direction [N]')
    grid on
    
    subplot(3,1,2)
    hold on
    plot(t, force.fullTrajectory.(EEselection)(1:length(t),2), lineColor{i}, 'DisplayName', EEselection);
    xlabel('time [s]')
    ylabel('force in y-direction [N]')
    grid on    

    subplot(3,1,3)
    hold on
    plot(t, force.fullTrajectory.(EEselection)(1:length(t),3), lineColor{i}, 'DisplayName', EEselection);
    xlabel('time [s]')
    ylabel('force in z-direction [N]')
    grid on    
end
if legCount > 1 % only display legend if there are multiple legs
    lgd = legend(p);
    lgd.FontSize = 14;
end
hold off

%% Plot x vs z position of EE for the final trajectory (after trimming and averaging).
figure(4)
set(gcf,'color','w')
title('Final end effector trajectory')
subplotRows = floor(legCount/2);
subplotRows(subplotRows<1) = 1;
subplotColumns = ceil(legCount/2);
startIndexFullTrajectory = round(length(data.(task).fullTrajectory.r.EEdes.LF)*removalRatioStart);
startIndexFullTrajectory(startIndexFullTrajectory<1) = 1;
endIndexFullTrajectory = round(length(data.(task).fullTrajectory.r.EEdes.LF)*(1-removalRatioEnd));

for i = 1:legCount
    EEselection = EEnames(i,:);
    subplot(subplotRows, subplotColumns,i)
    hold on
    plot(data.(task).fullTrajectory.r.EEdes.(EEselection)(startIndexFullTrajectory:endIndexFullTrajectory,1), data.(task).fullTrajectory.r.EEdes.(EEselection)(startIndexFullTrajectory:endIndexFullTrajectory,3), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
    plot(data.(task).(EEselection).r.EEdes(:,1), data.(task).(EEselection).r.EEdes(:,3), 'k', 'LineWidth', 2)
    axis equal
    xlabel('x position [m]')
    ylabel('z position [m]')
    title(EEselection)
    grid on
    hold off
end

%% End effector z motion with rectangle showing sampled range of data.
figure(5)
set(gcf,'color','w')
% EE Position 
subplot(2,2,[1 2])
    title('End effector position and force over sampled data range')
    hold on
    for i = 1:legCount
        EEselection = EEnames(i,:);
        p(i) = plot(t, data.(task).fullTrajectory.r.EEdes.(EEselection)(1:length(t),3), lineColor{i}, 'DisplayName', EEselection);
    end

    if legCount > 1 % only display legend if there are multiple legs
        lgd = legend(p);
        lgd.FontSize = 14;
    end

    ylabel('End effector z position [m]')
    % draw rectangle of sampled data range
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), 1.1*min(data.(task).fullTrajectory.r.EEdes.LF(:,3))];
    rectanglePos.width =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height = 1.3*(max(data.(task).fullTrajectory.r.EEdes.LF(:,3)) - min(data.(task).fullTrajectory.r.EEdes.LF(:,3)));
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 2);            
    clear rectanglePos
    xlabel('time [s]') 
    grid on
    hold off
    
% EE force 
subplot(2,2,[3 4])
    hold on
    for i = 1:legCount
        EEselection = EEnames(i,:);
        plot(t, data.(task).fullTrajectory.force.(EEselection)(1:length(t),3), lineColor{i}, 'DisplayName', EEselection);
    end
     xlabel('time [s]')
     ylabel('End effector z forces [N]')
    % draw rectangle of sampled data range
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), 1.1*min(data.(task).fullTrajectory.force.LF(:,3))];
    rectanglePos.width =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height = 1.3*(max(data.(task).fullTrajectory.force.LF(:,3)) - min(data.(task).fullTrajectory.force.LF(:,3)));
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 2);            
    grid on
    hold off
end