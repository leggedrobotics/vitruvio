% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(classSelection, taskSelection, classSelection2, taskSelection2, plotOptimizedLeg)

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
data = classSelection.(taskSelection);  % with interpolation
data2 = classSelection2.(taskSelection2); % no interpolation
dt = data.time(2) - data.time(1); % time is uniform so dt is constant
dt2 = data2.time(2) - data2.time(1);

% 
% if classSelection2 ~= 0
%      data2 = classSelection2.(taskSelection2);
%      dt = data2.time(2) - data2.time(1); % time is uniform so dt is constant
% else
%     data2 = data;
% end

xlimSpeedy       = [0, 0.8];
ylimSpeedyq      = [-90, 90];
ylimSpeedyqdot   = [-30 30];
ylimSpeedytorque   = [-200 200];
ylimSpeedypower  = [-2000 2000];
ylimSpeedyenergy = [0 200];

xlimANYmal       = [0, 1.5];
ylimANYmalq      = [-40 40];
ylimANYmalqdot   = [-10 10];
ylimANYmaltorque = [-20 20];
ylimANYmalpower  = [-20 20];
ylimANYmalenergy = [0 3];

xlimit.time   = xlimANYmal;
ylimit.q      = ylimANYmalq;
ylimit.qdot   = ylimANYmalqdot;
ylimit.torque   = ylimANYmaltorque;
ylimit.power  = ylimANYmalpower;
ylimit.energy = ylimANYmalenergy;

lineColour = 'b';
lineColourOpt = 'r';
faceColour = 'b';
faceColourOpt = 'r';

LineWidth = 1;
scatterSize = 12;

%% joint Position
figure('name', 'Joint Position', 'DefaultAxesFontSize', 10)
jointCount = length(data.LF.jointTorque(1,:));
for i = 1:4
    EEselection = EEnames(i,:);
    time =  0:dt:3*length(data.(EEselection).jointTorque)*dt-dt; 
    time2 = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2; 

    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
    data2.(EEselection).q = data2.(EEselection).q(:,:) - data2.(EEselection).q(1,:); % normalize so first point at zero

    scatter(time, [rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1))], lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim(xlimit.time)
    ylim(ylimit.q)
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    scatter(time, [rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2))],  scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
    legend('additional points', 'original points')   
    
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2))], lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim(xlimit.time)
    ylim(ylimit.q)
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    scatter(time, [rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        plot(time, [rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3))],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);   
        legend('initial design', 'optimized design')       
    end
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim(xlimit.time)
    ylim(ylimit.q)
    title([EEselection '\_KFE'])
    hold off
    
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        scatter(time, [rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
        if plotOptimizedLeg.(EEselection)
            plot(time, [rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4))],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')       
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [deg]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        scatter(time, [rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
        if plotOptimizedLeg.(EEselection)
            plot(time, [rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5))],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')    
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [deg]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title([EEselection '\_DFE'])
        hold off
    end
end

%% Joint velocity plots
figure('name', 'Joint Velocity', 'DefaultAxesFontSize', 10)
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;    
    time2 = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;    

    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    scatter(time,  [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
%     xlim([time(1), time(end)])
    xlim(xlimit.time)
    ylim(ylimit.qdot)
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    scatter(time, [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim(xlimit.time)
    ylim(ylimit.qdot)
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    scatter(time, [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')  
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim(xlimit.time)
    ylim(ylimit.qdot)
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        scatter(time, [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)    
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        scatter(time, [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
        grid on
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')  
        end        
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_DFE'])
        hold off
    end
end

%% Joint torque plots
figure('name', 'Joint Torque', 'DefaultAxesFontSize', 10)
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;
    time2 = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;    

    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    scatter(time, [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim(xlimit.time)
    ylim(ylimit.torque)
    title([EEselection '\_HAA'])
    hold off
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    scatter(time, [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end     
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim(xlimit.time)
    ylim(ylimit.torque)
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    scatter(time, [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end     
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim(xlimit.time)
    ylim(ylimit.torque)
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        scatter(time, [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')   
        end         
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        scatter(time, [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')  
        end        
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title([EEselection '\_DFE'])
        hold off
    end
end
%% Joint power plots
figure('name', 'Joint Power', 'DefaultAxesFontSize', 10)
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt;  
    time2 = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;    

    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    scatter(time, [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth); 
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim(xlimit.time)
    ylim(ylimit.power)
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    scatter(time, [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        plot(time, [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim(xlimit.time)
    ylim(ylimit.power)
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    scatter(time, [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')      
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Power [W]')
    xlim(xlimit.time)
    ylim(ylimit.power)
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        scatter(time, [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        scatter(time, [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')      
        end           
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
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
    for i = 1:length(data2.(EEselection).energy(:,1))-1
        data2.(EEselection).energy(i+1,:) = data2.(EEselection).energy(i,:) + data2.(EEselection).energy(i+1,:);
    end
end

figure('name', 'Joint Energy Consumption', 'DefaultAxesFontSize', 10)
for i = 1:4
    EEselection = EEnames(i,:);
    time = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt; 
    time2 = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;        
    %Hip abduction/adduction
    subplot(jointCount, 4, i);
    hold on
    scatter(time, [data.(EEselection).energy(:,1); data.(EEselection).energy(end,1) + data.(EEselection).energy(:,1); 2*data.(EEselection).energy(end,1) + data.(EEselection).energy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).energy(:,1); data2.(EEselection).energy(end,1) + data2.(EEselection).energy(:,1); 2*data2.(EEselection).energy(end,1) + data2.(EEselection).energy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);   
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).energyOpt(:,1); data.(EEselection).energyOpt(end,1) + data.(EEselection).energyOpt(:,1); 2*data.(EEselection).energyOpt(end,1) + data.(EEselection).energyOpt(:,1)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end    
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim(xlimit.time)
    ylim(ylimit.energy)
    title([EEselection '\_HAA'])
    hold off
    
    % Hip flexion/extension
    subplot(jointCount, 4, 4+i);
    hold on
    scatter(time, [data.(EEselection).energy(:,2); data.(EEselection).energy(end,2) + data.(EEselection).energy(:,2); 2*data.(EEselection).energy(end,2) + data.(EEselection).energy(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).energy(:,2); data2.(EEselection).energy(end,2) + data2.(EEselection).energy(:,2); 2*data2.(EEselection).energy(end,2) + data2.(EEselection).energy(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
    legend('additional points', 'original points')      
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).energyOpt(:,2); data.(EEselection).energyOpt(end,2) + data.(EEselection).energyOpt(:,2); 2*data.(EEselection).energyOpt(end,2) + data.(EEselection).energyOpt(:,2)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')   
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim(xlimit.time)
    ylim(ylimit.energy)
    title([EEselection '\_HFE'])
    hold off
    
    % Knee flexion/extension
    subplot(jointCount, 4, 8+i);
    hold on
    scatter(time, [data.(EEselection).energy(:,3); data.(EEselection).energy(end,3) + data.(EEselection).energy(:,3); 2*data.(EEselection).energy(end,3) + data.(EEselection).energy(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
    scatter(time2, [data2.(EEselection).energy(:,3); data2.(EEselection).energy(end,3) + data2.(EEselection).energy(:,3); 2*data2.(EEselection).energy(end,3) + data2.(EEselection).energy(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
    legend('additional points', 'original points')       
    if plotOptimizedLeg.(EEselection)
        scatter(time, [data.(EEselection).energyOpt(:,3); data.(EEselection).energyOpt(end,3) + data.(EEselection).energyOpt(:,3); 2*data.(EEselection).energyOpt(end,3) + data.(EEselection).energyOpt(:,3)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
        legend('initial design', 'optimized design')      
    end      
    grid on
    xlabel('Time [s]')
    ylabel('Energy [J]')
    xlim(xlimit.time)
    ylim(ylimit.energy)
    title([EEselection '\_KFE'])
    hold off
    if (jointCount == 4 || jointCount == 5) 
        subplot(jointCount, 4, 12+i);
        hold on
        scatter(time, [data.(EEselection).energy(:,4); data.(EEselection).energy(end,4) + data.(EEselection).energy(:,4); 2*data.(EEselection).energy(end,4) + data.(EEselection).energy(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).energy(:,4); data2.(EEselection).energy(end,4) + data2.(EEselection).energy(:,4); 2*data2.(EEselection).energy(end,4) + data2.(EEselection).energy(:,4)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).energyOpt(:,4); data.(EEselection).energyOpt(end,4) + data.(EEselection).energyOpt(:,4); 2*data.(EEselection).energyOpt(end,4) + data.(EEselection).energyOpt(:,4)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.energy)
        title([EEselection '\_AFE'])
        hold off
    end
    if jointCount == 5
        subplot(jointCount, 4, 16+i);
        hold on
        scatter(time, [data.(EEselection).energy(:,5); data.(EEselection).energy(end,5) + data.(EEselection).energy(:,5); 2*data.(EEselection).energy(end,5) + data.(EEselection).energy(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        scatter(time2, [data2.(EEselection).energy(:,5); data2.(EEselection).energy(end,5) + data2.(EEselection).energy(:,5); 2*data2.(EEselection).energy(end,5) + data2.(EEselection).energy(:,5)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
        if plotOptimizedLeg.(EEselection)
            scatter(time, [data.(EEselection).energyOpt(:,5); data.(EEselection).energyOpt(end,5) + data.(EEselection).energyOpt(:,5); 2*data.(EEselection).energyOpt(end,5) + data.(EEselection).energyOpt(:,5)],lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);
            legend('initial design', 'optimized design')      
        end           
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.energy)
        title([EEselection '\_DFE'])
        hold off
    end
end