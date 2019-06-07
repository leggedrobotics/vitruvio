% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(classSelection, taskSelection)
EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
data = classSelection.(taskSelection);
figure()
dt = 0.01;
jointCount = length(data.LF.jointTorque(1,:));
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    plot(time, [rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1))]);
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2))]);
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
    title([EEselection '\_HFE'])
    hold off
    % Knee flexion/extension
    subplot(jointCount, 4, 8 + i);
    hold on
    plot(time, [rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3))]);
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12 + i);
        hold on
        plot(time, [rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4))]);
        grid on
        xlabel('Time [s]')
        ylabel('Position [deg]')
        xlim([time(1),time(end)])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16 + i);
        hold on
        plot(time, [rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5))]);
        grid on
        xlabel('Time [s]')
        ylabel('Position [deg]')
        xlim([time(1),time(end)])
        title([EEselection '\_DFE'])
        hold off
    end
end

%% Joint velocity plots
figure()
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    plot(time, [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)]);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)]);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([EEselection '\_HFE'])
    hold off
    % Knee flexion/extension
    subplot(jointCount, 4, 8 + i);
    hold on
    plot(time, [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)]);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)]);
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim([time(1),time(end)])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        plot(time, [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)]);
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim([time(1),time(end)])
        title([EEselection '\_DFE'])
        hold off
    end
end

%% Joint torque plots
figure()
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    plot(time, [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)]);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)]);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([EEselection '\_HFE'])
    hold off
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)]);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)]);
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim([time(1),time(end)])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        plot(time, [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)]);
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim([time(1),time(end)])
        title([EEselection '\_DFE'])
        hold off
    end
end
%% Joint power plots
figure()
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    plot(time, [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)]);
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)]);
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    title([EEselection '\_HFE'])
    hold off
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)]);
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)]);
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim([time(1),time(end)])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        plot(time, [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)]);
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim([time(1),time(end)])
        title([EEselection '\_DFE'])
        hold off
    end
end
end

