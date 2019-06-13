% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(classSelection, taskSelection, plotOptimizedLeg)
EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
data = classSelection.(taskSelection);
dt = 0.01;

figure()
jointCount = length(data.LF.jointTorque(1,:));
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
    
    plot(time, [rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1))],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1))], 'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
%     ylim([-5, 5])
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2))], 'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2))], 'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
%     ylim([-30, 30])
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3))],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3))],'b', 'LineWidth', 2);   
        legend('initial design', 'optimized design')       
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
%     ylim([-35, 35])
    title([EEselection '\_KFE'])
    hold off
    
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4))],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4))],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')       
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [deg]')
        xlim([time(1),time(end)])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        plot(time, [rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5))],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5))],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')    
        end
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
    plot(time, [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    ylim([-1, 1])
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    ylim([-4, 3]);
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')  
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    ylim([-6, 6]);
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')   
        end
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
        plot(time, [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)],'r', 'LineWidth', 2);
        grid on
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')  
        end        
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
    plot(time, [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    ylim([-20, 5])
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end     
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    ylim([-20, 15]);
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end     
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    ylim([-30, 40]);
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')   
        end         
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
        plot(time, [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')  
        end        
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
    plot(time, [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)],'b', 'LineWidth', 2); 
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    ylim([-3, 1.5])
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    ylim([-40, 35]);

    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')      
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim([time(1),time(end)])
    ylim([-15, 15]);
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')   
        end      
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
        plot(time, [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')      
        end           
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim([time(1),time(end)])
        title([EEselection '\_DFE'])
        hold off
    end
end

%% Joint energy consumpton plots
% compute the consumed energy by adding previous terms 
for i = 1:4
    EEselection = EEnames(i,:);
    for i = 1:length(data.(EEselection).energy(:,1))-1
        data.(EEselection).energy(i+1,:) = data.(EEselection).energy(i,:) + data.(EEselection).energy(i+1,:);
        if plotOptimizedLeg.(EEselection)
            data.(EEselection).energyOpt(i+1,:) = data.(EEselection).energyOpt(i,:) + data.(EEselection).energyOpt(i+1,:);
        end
    end
end

figure()
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    plot(time, [data.(EEselection).energy(:,1); data.(EEselection).energy(end,1) + data.(EEselection).energy(:,1); 2*data.(EEselection).energy(end,1) + data.(EEselection).energy(:,1)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).energyOpt(:,1); data.(EEselection).energyOpt(end,1) + data.(EEselection).energyOpt(:,1); 2*data.(EEselection).energyOpt(end,1) + data.(EEselection).energyOpt(:,1)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim([time(1),time(end)])
    ylim([0,1])
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    plot(time, [data.(EEselection).energy(:,2); data.(EEselection).energy(end,2) + data.(EEselection).energy(:,2); 2*data.(EEselection).energy(end,2) + data.(EEselection).energy(:,2)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).energyOpt(:,2); data.(EEselection).energyOpt(end,2) + data.(EEselection).energyOpt(:,2); 2*data.(EEselection).energyOpt(end,2) + data.(EEselection).energyOpt(:,2)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')   
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim([time(1),time(end)])
    ylim([0,6])
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    plot(time, [data.(EEselection).energy(:,3); data.(EEselection).energy(end,3) + data.(EEselection).energy(:,3); 2*data.(EEselection).energy(end,3) + data.(EEselection).energy(:,3)],'r', 'LineWidth', 2);
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).energyOpt(:,3); data.(EEselection).energyOpt(end,3) + data.(EEselection).energyOpt(:,3); 2*data.(EEselection).energyOpt(end,3) + data.(EEselection).energyOpt(:,3)],'b', 'LineWidth', 2);
        legend('initial design', 'optimized design')      
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim([time(1),time(end)])
    ylim([0,10])
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        plot(time, [data.(EEselection).energy(:,4); data.(EEselection).energy(end,4) + data.(EEselection).energy(:,4); 2*data.(EEselection).energy(end,4) + data.(EEselection).energy(:,4)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).energyOpt(:,4); data.(EEselection).energyOpt(end,4) + data.(EEselection).energyOpt(:,4); 2*data.(EEselection).energyOpt(end,4) + data.(EEselection).energyOpt(:,4)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim([time(1),time(end)])
        ylim([0,4])
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        plot(time, [data.(EEselection).energy(:,5); data.(EEselection).energy(end,5) + data.(EEselection).energy(:,5); 2*data.(EEselection).energy(end,5) + data.(EEselection).energy(:,5)],'r', 'LineWidth', 2);
        if plotOptimizedLeg.(EEselection)
            plot(time, [data.(EEselection).energyOpt(:,5); data.(EEselection).energyOpt(end,5) + data.(EEselection).energyOpt(:,5); 2*data.(EEselection).energyOpt(end,5) + data.(EEselection).energyOpt(:,5)],'b', 'LineWidth', 2);
            legend('initial design', 'optimized design')      
        end           
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim([time(1),time(end)])
        ylim([0,4])
        title([EEselection '\_DFE'])
        hold off
    end
end
end
