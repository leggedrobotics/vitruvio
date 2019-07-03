% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(classSelection, taskSelection, classSelection2, taskSelection2, plotOptimizedLeg, plotType)

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];

if isempty(classSelection2)
     plotDataSet2 = false;
else
     plotDataSet2 = true;
     data2 = classSelection2.(taskSelection2); % no interpolation
     dt2 = data2.time(2) - data2.time(1);
end

data = classSelection.(taskSelection);  % with interpolation
dt = data.time(2) - data.time(1); % time is uniform so dt is constant

for i = 1:4
    EEselection = EEnames(i,:);
    time.(EEselection) = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt; % caputre motion over three cycles
    if plotDataSet2
        time2.(EEselection) = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;    
    end
end
    
xlimSpeedy       = [0, 3];
ylimSpeedyq      = [-90, 90];
ylimSpeedyqdot   = [-30 30];
ylimSpeedytorque   = [-200 200];
ylimSpeedypower  = [-2000 2000];
ylimSpeedyMechEnergy = [0 200];
ylimSpeedyElecEnergy = [0 200];

xlimANYmal       = [0, 5];
ylimANYmalq      = [-40 40];
ylimANYmalqdot   = [-10 10];
ylimANYmaltorque = [-20 20];
ylimANYmalpower  = [-35 35];
ylimANYmalMechEnergy = [0 10];
ylimANYmalElecEnergy = [0 10];

xlimit.time   = xlimANYmal;
ylimit.q      = ylimANYmalq;
ylimit.qdot   = ylimANYmalqdot;
ylimit.torque   = ylimANYmaltorque;
ylimit.power  = ylimANYmalpower;
ylimit.mechEnergy = ylimANYmalMechEnergy;
ylimit.elecEnergy = ylimANYmalElecEnergy;

lineColour = 'r';
faceColour = 'r';
lineColourOpt = 'b';
faceColourOpt = 'b';

LineWidth = 2;
scatterSize = 12;

%% SCATTER PLOT %% 
if isequal(plotType, 'scatterPlot') 
    %% joint Position
    figure('name', 'Joint Position', 'DefaultAxesFontSize', 10)
    jointCount = length(data.LF.jointTorque(1,:));
    for i = 1:4
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
        if plotDataSet2
            data2.(EEselection).q = data2.(EEselection).q(:,:) - data2.(EEselection).q(1,:); % normalize so first point at zero
        end
        scatter(time.(EEselection), [rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2    
            scatter(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2))],  scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2    
            scatter(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')   
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2        
            scatter(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3))],  scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);   
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
            scatter(time.(EEselection), [rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2    
                scatter(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5))], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2  
                scatter(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5))], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        scatter(time.(EEselection),  [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2  
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, 4, 4+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            grid on
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],  scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt); 
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
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
            scatter(time.(EEselection), [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
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
    figure('name', 'Mechanical Energy D', 'DefaultAxesFontSize', 10)
    for i = 1:4
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,1); data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1); 2*data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,1); data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1); 2*data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);   
            legend('with interpolated points', 'original points')
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,1); data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1); 2*data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, 4, 4+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,2); data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2); 2*data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,2); data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2); 2*data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,2); data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2); 2*data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, 4, 8+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,3); data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3); 2*data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,3); data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3); 2*data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,3); data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3); 2*data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')      
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, 4, 12+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,4); data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4); 2*data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,4); data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4); 2*data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,4); data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4); 2*data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('initial design', 'optimized design')   
            end      
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title([EEselection '\_AFE'])
            hold off
        end
        if jointCount == 5
            subplot(jointCount, 4, 16+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,5); data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5); 2*data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,5); data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5); 2*data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,5); data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5); 2*data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('initial design', 'optimized design')      
            end           
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title([EEselection '\_DFE'])
            hold off
        end
    end
else 
%% Line plot
        %% joint Position
    figure('name', 'Joint Position', 'DefaultAxesFontSize', 10)
    jointCount = length(data.LF.jointTorque(1,:));
    for i = 1:4
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
        if plotDataSet2
            data2.(EEselection).q = data2.(EEselection).q(:,:) - data2.(EEselection).q(1,:); % normalize so first point at zero
        end
        plot(time.(EEselection), [rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1)); rad2deg(data.(EEselection).q(:,1))], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2    
            plot(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1)); rad2deg(data2.(EEselection).q(:,1))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1)); rad2deg(data.(EEselection).qOpt(:,1))], lineColourOpt, 'LineWidth', LineWidth);
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
        plot(time.(EEselection), [rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2)); rad2deg(data.(EEselection).q(:,2))],  lineColour, 'LineWidth', LineWidth);
        if plotDataSet2    
            plot(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2)); rad2deg(data2.(EEselection).q(:,2))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')   
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2)); rad2deg(data.(EEselection).qOpt(:,2))], lineColourOpt, 'LineWidth', LineWidth);
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
        plot(time.(EEselection), [rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3)); rad2deg(data.(EEselection).q(:,3))], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2        
            plot(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3)); rad2deg(data2.(EEselection).q(:,3))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3)); rad2deg(data.(EEselection).qOpt(:,3))],  lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);   
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
            plot(time.(EEselection), [rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4)); rad2deg(data.(EEselection).q(:,4))], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                plot(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4)); rad2deg(data.(EEselection).qOpt(:,4))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5)); rad2deg(data.(EEselection).q(:,5))], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2  
                plot(time2.(EEselection), [rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4)); rad2deg(data2.(EEselection).q(:,4))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5)); rad2deg(data.(EEselection).qOpt(:,5))], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        plot(time.(EEselection),  [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2  
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, 4, 4+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
        plot(time.(EEselection), [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            grid on
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')  
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time, [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],  lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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

        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt); 
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
        plot(time.(EEselection), [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
        plot(time.(EEselection), [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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
            plot(time.(EEselection), [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
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

    figure('name', 'Joint Energy Consumption', 'DefaultAxesFontSize', 10)
    for i = 1:4
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, 4, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,1); data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1); 2*data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,1); data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1); 2*data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1)], lineColour, 'LineWidth', LineWidthOpt,'MarkerFaceColor', faceColourOpt);   
            legend('with interpolated points', 'original points')
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,1); data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1); 2*data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, 4, 4+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,2); data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2); 2*data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,2); data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2); 2*data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,2); data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2); 2*data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, 4, 8+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,3); data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3); 2*data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,3); data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3); 2*data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')       
        end
        if plotOptimizedLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,3); data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3); 2*data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('initial design', 'optimized design')      
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, 4, 12+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).mechEnergy(:,4); data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4); 2*data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,4); data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4); 2*data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4)], LineColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,4); data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4); 2*data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth);
                legend('initial design', 'optimized design')   
            end      
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title([EEselection '\_AFE'])
            hold off
        end
        if jointCount == 5
            subplot(jointCount, 4, 16+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).mechEnergy(:,5); data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5); 2*data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,5); data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5); 2*data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5)], LineColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if plotOptimizedLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,5); data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5); 2*data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth);
                legend('initial design', 'optimized design')      
            end           
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title([EEselection '\_DFE'])
            hold off
        end
    end
end