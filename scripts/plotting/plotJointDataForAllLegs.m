% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(classSelection, taskSelection, classSelection2, taskSelection2, optimizeLeg, plotType)

legCount = classSelection.(taskSelection).basicProperties.legCount;
EEnames  = classSelection.(taskSelection).basicProperties.EEnames;

%% Second data set
if isempty(classSelection2)
     plotDataSet2 = false;
else
     plotDataSet2 = true;
     data2 = classSelection2.(taskSelection2); % no interpolation
     dt2 = data2.time(2) - data2.time(1);
end

%% Load in data and plot options
data = classSelection.(taskSelection);  % with interpolation
dt = data.time(2) - data.time(1); % time is uniform so dt is constant

for i = 1:legCount
    EEselection = EEnames(i,:);
    time.(EEselection) = 0:dt:3*length(data.(EEselection).jointTorque)*dt-dt; % caputre motion over three cycles
    if plotDataSet2
        time2.(EEselection) = 0:dt2:3*length(data2.(EEselection).jointTorque)*dt2-dt2;    
    end
end
    
%% Get y limits
% initialize vectors for nominal limits to be empty
qMax          = []; qMin          = [];
qdotMax       = []; qdotMin       = [];
torqueMax     = []; torqueMin     = [];
powerMax      = []; powerMin      = [];
mechEnergyMax = []; mechEnergyMin = [];
elecEnergyMax = []; elecEnergyMin = [];

% initialize vectors for optimal limits to be empty
qMaxOpt          = []; qMinOpt          = [];
qdotMaxOpt       = []; qdotMinOpt       = [];
torqueMaxOpt     = []; torqueMinOpt     = [];
powerMaxOpt      = []; powerMinOpt      = [];
mechEnergyMaxOpt = []; mechEnergyMinOpt = [];
elecEnergyMaxOpt = []; elecEnergyMinOpt = [];

for i = 1:legCount
    EEselection = EEnames(i,:);
    maxq.(EEselection) = max(max(data.(EEselection).q - data.(EEselection).q(1,:)));
    minq.(EEselection) = min(min(data.(EEselection).q - data.(EEselection).q(1,:)));

    maxqdot.(EEselection) = max(max(data.(EEselection).qdot));
    minqdot.(EEselection) = min(min(data.(EEselection).qdot));

    maxTorque.(EEselection) = max(max(data.(EEselection).jointTorque));
    minTorque.(EEselection) = min(min(data.(EEselection).jointTorque));

    maxPower.(EEselection) = max(max(data.(EEselection).jointPower));
    minPower.(EEselection) = min(min(data.(EEselection).jointPower));

    maxMechEnergy.(EEselection) = max(max(data.(EEselection).mechEnergy));
    minMechEnergy.(EEselection) = min(min(data.(EEselection).mechEnergy));

    maxElecEnergy.(EEselection) = max(max(data.(EEselection).elecEnergy));
    minElecEnergy.(EEselection) = min(min(data.(EEselection).elecEnergy));
    
    % fill in the vectors for optimal limits
    qMax = [qMax, maxq.(EEselection)];                            qMin = [qMin, minq.(EEselection)];
    qdotMax = [qdotMax, maxqdot.(EEselection)];                   qdotMin = [qdotMin, minqdot.(EEselection)];
    torqueMax = [torqueMax, maxTorque.(EEselection)];             torqueMin = [torqueMin, minTorque.(EEselection)];
    powerMax = [powerMax, maxPower.(EEselection)];                powerMin = [powerMin, minPower.(EEselection)];
    mechEnergyMax = [mechEnergyMax, maxMechEnergy.(EEselection)]; mechEnergyMin = [mechEnergyMin, minMechEnergy.(EEselection)];
    elecEnergyMax = [elecEnergyMax, maxElecEnergy.(EEselection)]; elecEnergyMin = [elecEnergyMin, minElecEnergy.(EEselection)];
    
    %% Get y limits opt
    if optimizeLeg.(EEselection)

        maxqOpt.(EEselection) = max(max(data.(EEselection).qOpt - data.(EEselection).qOpt(1,:)));
        minqOpt.(EEselection) = min(min(data.(EEselection).qOpt - data.(EEselection).qOpt(1,:)));

        maxqdotOpt.(EEselection) = max(max(data.(EEselection).qdotOpt));
        minqdotOpt.(EEselection) = min(min(data.(EEselection).qdotOpt));

        maxTorqueOpt.(EEselection) = max(max(data.(EEselection).jointTorqueOpt));
        minTorqueOpt.(EEselection) = min(min(data.(EEselection).jointTorqueOpt));

        maxPowerOpt.(EEselection) = max(max(data.(EEselection).jointPowerOpt));
        minPowerOpt.(EEselection) = min(min(data.(EEselection).jointPowerOpt));

        maxMechEnergyOpt.(EEselection) = max(max(data.(EEselection).mechEnergyOpt));
        minMechEnergyOpt.(EEselection) = min(min(data.(EEselection).mechEnergyOpt));

        maxElecEnergyOpt.(EEselection) = max(max(data.(EEselection).elecEnergyOpt));
        minElecEnergyOpt.(EEselection) = min(min(data.(EEselection).elecEnergyOpt));
        
        % fill in the vectors for nominal limits
        qMaxOpt          = [qMaxOpt, maxqOpt.(EEselection)];                   qMin             = [qMinOpt, minqOpt.(EEselection)];
        qdotMaxOpt       = [qdotMaxOpt, maxqdotOpt.(EEselection)];             qdotMinOpt       = [qdotMinOpt, minqdotOpt.(EEselection)];
        torqueMaxOpt     = [torqueMaxOpt, maxTorqueOpt.(EEselection)];         torqueMinOpt     = [torqueMinOpt, minTorqueOpt.(EEselection)];
        powerMaxOpt      = [powerMaxOpt, maxPowerOpt.(EEselection)];           powerMinOpt      = [powerMinOpt, minPowerOpt.(EEselection)];
        mechEnergyMaxOpt = [mechEnergyMaxOpt, maxMechEnergyOpt.(EEselection)]; mechEnergyMinOpt = [mechEnergyMinOpt, minMechEnergyOpt.(EEselection)];
        elecEnergyMaxOpt = [elecEnergyMaxOpt, maxElecEnergyOpt.(EEselection)]; elecEnergyMinOpt = [elecEnergyMinOpt, minElecEnergyOpt.(EEselection)];
    end
end

xlimit.time       = [0, time.LF(end)];                                          % buffer terms
ylimit.q          = [min([qMin, qMinOpt]), max([qMax, qMaxOpt])]                     + [-0.2*abs(min(qMin)), 0.2*abs(max(qMax))];
ylimit.qdot       = [min([qdotMin, qdotMinOpt]), max([qdotMax, qdotMaxOpt])]         + [-0.2*abs(min(qdotMin)), 0.2*abs(max(qdotMax))];
ylimit.torque     = [min([torqueMin, torqueMinOpt]), max([torqueMax, torqueMaxOpt])] + [-0.2*abs(min(torqueMin)), 0.2*abs(max(torqueMax))];
ylimit.power      = [min([powerMin, powerMinOpt]), max([powerMax, powerMaxOpt])]     + [-0.2*abs(min(powerMin)), 0.2*abs(max(powerMax))];
ylimit.mechEnergy = [0, 3*max([mechEnergyMax mechEnergyMaxOpt])]; 
ylimit.elecEnergy = [0, 3*max([elecEnergyMax  elecEnergyMaxOpt])];


lineColour = 'r';
faceColour = 'r';
lineColourOpt = 'b';
faceColourOpt = 'b';

LineWidth = 1;
scatterSize = 12;

%% SCATTER PLOT %% 
if isequal(plotType, 'scatterPlot') 
    %% joint Position
    figure('name', 'Joint Position', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    jointCount = length(data.LF.jointTorque(1,:));
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
        if plotDataSet2
            data2.(EEselection).q = data2.(EEselection).q(:,:) - data2.(EEselection).q(1,:); % normalize so first point at zero
        end
        scatter(time.(EEselection), [data.(EEselection).q(:,1); data.(EEselection).q(:,1); data.(EEselection).q(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2    
            scatter(time2.(EEselection), [data2.(EEselection).q(:,1); data2.(EEselection).q(:,1); data2.(EEselection).q(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            data.(EEselection).qOpt = data.(EEselection).qOpt - data.(EEselection).qOpt(1,:); % normalize so first point at zero
            scatter(time.(EEselection), [data.(EEselection).qOpt(:,1); data.(EEselection).qOpt(:,1); data.(EEselection).qOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized')
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).q(:,2); data.(EEselection).q(:,2); data.(EEselection).q(:,2)],  scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2    
            scatter(time2.(EEselection), [data2.(EEselection).q(:,2); data2.(EEselection).q(:,2); data2.(EEselection).q(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qOpt(:,2); data.(EEselection).qOpt(:,2); data.(EEselection).qOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt);
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).q(:,3); data.(EEselection).q(:,3); data.(EEselection).q(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2        
            scatter(time2.(EEselection), [data2.(EEselection).q(:,3); data2.(EEselection).q(:,3); data2.(EEselection).q(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qOpt(:,3); data.(EEselection).qOpt(:,3); data.(EEselection).qOpt(:,3)],  scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title([EEselection '\_KFE'])
        hold off

        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).q(:,4); data.(EEselection).q(:,4); data.(EEselection).q(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2    
                scatter(time2.(EEselection), [data2.(EEselection).q(:,4); data2.(EEselection).q(:,4); data2.(EEselection).q(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qOpt(:,4); data.(EEselection).qOpt(:,4); data.(EEselection).qOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            end
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.q)
            title([EEselection '\_AFE'])
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).q(:,5); data.(EEselection).q(:,5); data.(EEselection).q(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2  
                scatter(time2.(EEselection), [data2.(EEselection).q(:,4); data2.(EEselection).q(:,5); data2.(EEselection).q(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qOpt(:,5); data.(EEselection).qOpt(:,5); data.(EEselection).qOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            end
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.q)
            title([EEselection '\_DFE'])
            hold off
        end
    end
   export_fig results.pdf -nocrop -append

    %% Joint velocity plots
    figure('name', 'Joint Velocity', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        scatter(time.(EEselection),  [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2  
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized')   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3)],scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    
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
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            grid on
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('nominal', 'optimized')  
            end        
            xlabel('Time [s]')
            ylabel('Velocity [rad/s]')
            xlim(xlimit.time)
            ylim(ylimit.qdot)
            title([EEselection '\_DFE'])
            hold off
        end
    end
   export_fig results.pdf -nocrop -append

    %% Joint torque plots
    figure('name', 'Joint Torque', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title([EEselection '\_HAA'])
        hold off
        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);

        end     
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);

        end     
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            scatter(time, [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],  scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    
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
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('nominal', 'optimized')  
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
   export_fig results.pdf -nocrop -append
    
    %% Joint power plots
    figure('name', 'Joint Power', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt); 
            legend('nominal', 'optimized')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);

        end      
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    
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
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
       
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
   export_fig results.pdf -nocrop -append

    %% Joint energy consumpton plots
    % compute the consumed energy by adding previous terms 
    figure('name', 'Mechanical Energy D', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,1); data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1); 2*data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,1); data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1); 2*data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1)], scatterSize, lineColour, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);   
            legend('with interpolated points', 'original points')
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,1); data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1); 2*data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HAA'])
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,2); data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2); 2*data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,2); data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2); 2*data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,2); data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2); 2*data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);

        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_HFE'])
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,3); data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3); 2*data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
        if plotDataSet2
            scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,3); data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3); 2*data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,3); data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3); 2*data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
   
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title([EEselection '\_KFE'])
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,4); data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4); 2*data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,4); data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4); 2*data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,4); data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4); 2*data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
    
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
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            scatter(time.(EEselection), [data.(EEselection).mechEnergy(:,5); data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5); 2*data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5)], scatterSize, lineColour, 'MarkerFaceColor', faceColour);
            if plotDataSet2
                scatter(time2.(EEselection), [data2.(EEselection).mechEnergy(:,5); data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5); 2*data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5)], lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                scatter(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,5); data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5); 2*data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5)], scatterSize, lineColourOpt, 'MarkerFaceColor', faceColourOpt,'MarkerFaceColor', faceColourOpt);
       
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
   export_fig results.pdf -nocrop -append
    
else    
%% Line plot
    %% joint Position
    figure('name', 'Joint Position', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    jointCount = length(data.LF.jointTorque(1,:));
    for i = 1:legCount
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        data.(EEselection).q = data.(EEselection).q(:,:) - data.(EEselection).q(1,:); % normalize so first point at zero
        if plotDataSet2
            data2.(EEselection).q = data2.(EEselection).q(:,:) - data2.(EEselection).q(1,:); % normalize so first point at zero
        end
        plot(time.(EEselection), [data.(EEselection).q(:,1); data.(EEselection).q(:,1); data.(EEselection).q(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2    
            plot(time2.(EEselection), [data2.(EEselection).q(:,1); data2.(EEselection).q(:,1); data2.(EEselection).q(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            data.(EEselection).qOpt = data.(EEselection).qOpt - data.(EEselection).qOpt(1,:); % normalize so first point at zero
            plot(time.(EEselection), [data.(EEselection).qOpt(:,1); data.(EEselection).qOpt(:,1); data.(EEselection).qOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth);
            legend('nominal', 'optimized')
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title({EEselection, 'q_{HAA}'})
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).q(:,2); data.(EEselection).q(:,2); data.(EEselection).q(:,2)],  lineColour, 'LineWidth', LineWidth);
        if plotDataSet2    
            plot(time2.(EEselection), [data2.(EEselection).q(:,2); data2.(EEselection).q(:,2); data2.(EEselection).q(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')   
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qOpt(:,2); data.(EEselection).qOpt(:,2); data.(EEselection).qOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth);
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title('q_{HFE}')
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).q(:,3); data.(EEselection).q(:,3); data.(EEselection).q(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2        
            plot(time2.(EEselection), [data2.(EEselection).q(:,3); data2.(EEselection).q(:,3); data2.(EEselection).q(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qOpt(:,3); data.(EEselection).qOpt(:,3); data.(EEselection).qOpt(:,3)],  lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);   
    
        end
        grid on
        xlabel('Time [s]')
        ylabel('Position [rad]')
        xlim(xlimit.time)
        ylim(ylimit.q)
        title(' q_{KFE}')
        hold off

        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).q(:,4); data.(EEselection).q(:,4); data.(EEselection).q(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                plot(time2.(EEselection), [data2.(EEselection).q(:,4); data2.(EEselection).q(:,4); data2.(EEselection).q(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qOpt(:,4); data.(EEselection).qOpt(:,4); data.(EEselection).qOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        
            end
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.q)
            title([EEselection, ' q_{AFE}'])
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).q(:,5); data.(EEselection).q(:,5); data.(EEselection).q(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2  
                plot(time2.(EEselection), [data2.(EEselection).q(:,4); data2.(EEselection).q(:,4); data2.(EEselection).q(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qOpt(:,5); data.(EEselection).qOpt(:,5); data.(EEselection).qOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
     
            end
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.q)
            title('q_{DFE}')
            hold off
        end
    end
   export_fig results.pdf -nocrop -append

    %% Joint velocity plots
    figure('name', 'Joint Velocity', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        plot(time.(EEselection),  [data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1); data.(EEselection).qdot(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2  
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1); data2.(EEselection).qdot(:,1)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1); data.(EEselection).qdotOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized')   
        end
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title({EEselection, '\omega_{HAA}'})
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2); data.(EEselection).qdot(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2); data2.(EEselection).qdot(:,2)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2); data.(EEselection).qdotOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title('\omega_{HFE}')
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3); data.(EEselection).qdot(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3); data2.(EEselection).qdot(:,3)],lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3); data.(EEselection).qdotOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Velocity [rad/s]')
        xlim(xlimit.time)
        ylim(ylimit.qdot)
        title('\omega_{KFE}')
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4); data.(EEselection).qdot(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4); data2.(EEselection).qdot(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4); data.(EEselection).qdotOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end
            grid on
            xlabel('Time [s]')
            ylabel('Velocity [rad/s]')
            xlim(xlimit.time)
            ylim(ylimit.qdot)    
            title('\omega_{AFE}')
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5); data.(EEselection).qdot(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5); data2.(EEselection).qdot(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);            
                legend('with interpolated points', 'original points')
            end
            grid on
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5); data.(EEselection).qdotOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end        
            xlabel('Time [s]')
            ylabel('Velocity [rad/s]')
            xlim(xlimit.time)
            ylim(ylimit.qdot)
            title('\omega_{DFE}')
            hold off
        end
    end
   export_fig results.pdf -nocrop -append

    %% Joint torque plots
    figure('name', 'Joint Torque', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1); data.(EEselection).jointTorque(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1); data2.(EEselection).jointTorque(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1); data.(EEselection).jointTorqueOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized', 'location', 'east')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title({EEselection, '\tau_{HAA}'})
        hold off
        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2); data.(EEselection).jointTorque(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2); data2.(EEselection).jointTorque(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')  
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2); data.(EEselection).jointTorqueOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end     
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title('\tau_{HFE}')
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3); data.(EEselection).jointTorque(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3); data2.(EEselection).jointTorque(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3); data.(EEselection).jointTorqueOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end     
        grid on
        xlabel('Time [s]')
        ylabel('Torque [Nm]')
        xlim(xlimit.time)
        ylim(ylimit.torque)
        title('\tau_{KFE}')
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4); data.(EEselection).jointTorque(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4); data2.(EEselection).jointTorque(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4); data.(EEselection).jointTorqueOpt(:,4)],  lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end         
            grid on
            xlabel('Time [s]')
            ylabel('Torque [Nm]')
            xlim(xlimit.time)
            ylim(ylimit.torque)
            title('\tau_{AFE}')
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5); data.(EEselection).jointTorque(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5); data2.(EEselection).jointTorque(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5); data.(EEselection).jointTorqueOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end        
            grid on
            xlabel('Time [s]')
            ylabel('Torque [Nm]')
            xlim(xlimit.time)
            ylim(ylimit.torque)
            title('\tau_{DFE}')
            hold off
        end
    end
       export_fig results.pdf -nocrop -append

    %% Joint power plots
    figure('name', 'Joint Power', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);

        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1); data.(EEselection).jointPower(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1); data2.(EEselection).jointPower(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);    
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1); data.(EEselection).jointPowerOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt); 
            legend('nominal', 'optimized', 'location', 'east')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title({EEselection, 'P_{mech HAA}'})
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2); data.(EEselection).jointPower(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2); data2.(EEselection).jointPower(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')      
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2); data.(EEselection).jointPowerOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title('P_{mech HFE}')
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3); data.(EEselection).jointPower(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3); data2.(EEselection).jointPower(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3); data.(EEselection).jointPowerOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Power [W]')
        xlim(xlimit.time)
        ylim(ylimit.power)
        title('P_{mech DFE}')
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4); data.(EEselection).jointPower(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4); data2.(EEselection).jointPower(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);        
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4); data.(EEselection).jointPowerOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end      
            grid on
            xlabel('Time [s]')
            ylabel('Power [W]')
            xlim(xlimit.time)
            ylim(ylimit.power)
            title('P_{mech AFE}')
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5); data.(EEselection).jointPower(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5); data2.(EEselection).jointPower(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5); data.(EEselection).jointPowerOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end           
            grid on
            xlabel('Time [s]')
            ylabel('Power [W]')
            xlim(xlimit.time)
            ylim(ylimit.power)
            title('P_{mech DFE}')
            hold off
        end
    end
       export_fig results.pdf -nocrop -append
 
    figure('name', 'Joint Energy Consumption', 'DefaultAxesFontSize', 10)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);
        %Hip abduction/adduction
        subplot(jointCount, legCount, i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,1); data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1); 2*data.(EEselection).mechEnergy(end,1) + data.(EEselection).mechEnergy(:,1)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,1); data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1); 2*data2.(EEselection).mechEnergy(end,1) + data2.(EEselection).mechEnergy(:,1)], lineColour, 'LineWidth', LineWidthOpt,'MarkerFaceColor', faceColourOpt);   
            legend('with interpolated points', 'original points')
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,1); data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1); 2*data.(EEselection).mechEnergyOpt(end,1) + data.(EEselection).mechEnergyOpt(:,1)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            legend('nominal', 'optimized', 'location', 'east')   
        end    
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title({EEselection, 'E_{mech HAA}'})
        hold off

        % Hip flexion/extension
        subplot(jointCount, legCount, legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,2); data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2); 2*data.(EEselection).mechEnergy(end,2) + data.(EEselection).mechEnergy(:,2)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,2); data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2); 2*data2.(EEselection).mechEnergy(end,2) + data2.(EEselection).mechEnergy(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,2); data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2); 2*data.(EEselection).mechEnergyOpt(end,2) + data.(EEselection).mechEnergyOpt(:,2)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title('E_{mech HFE}')
        hold off

        % Knee flexion/extension
        subplot(jointCount, legCount, 2*legCount+i);
        hold on
        plot(time.(EEselection), [data.(EEselection).mechEnergy(:,3); data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3); 2*data.(EEselection).mechEnergy(end,3) + data.(EEselection).mechEnergy(:,3)], lineColour, 'LineWidth', LineWidth);
        if plotDataSet2
            plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,3); data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3); 2*data2.(EEselection).mechEnergy(end,3) + data2.(EEselection).mechEnergy(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);       
            legend('with interpolated points', 'original points')       
        end
        if optimizeLeg.(EEselection)
            plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,3); data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3); 2*data.(EEselection).mechEnergyOpt(end,3) + data.(EEselection).mechEnergyOpt(:,3)], lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
        end      
        grid on
        xlabel('Time [s]')
        ylabel('Energy [J]')
        xlim(xlimit.time)
        ylim(ylimit.mechEnergy)
        title('E_{mech KFE}')
        hold off
        if (jointCount == 4 || jointCount == 5) 
            subplot(jointCount, legCount, 3*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).mechEnergy(:,4); data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4); 2*data.(EEselection).mechEnergy(end,4) + data.(EEselection).mechEnergy(:,4)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,4); data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4); 2*data2.(EEselection).mechEnergy(end,4) + data2.(EEselection).mechEnergy(:,4)], LineColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,4); data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4); 2*data.(EEselection).mechEnergyOpt(end,4) + data.(EEselection).mechEnergyOpt(:,4)], lineColourOpt, 'LineWidth', LineWidth);
            end      
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title('E_{mech AFE}')
            hold off
        end
        if jointCount == 5
            subplot(jointCount, legCount, 4*legCount+i);
            hold on
            plot(time.(EEselection), [data.(EEselection).mechEnergy(:,5); data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5); 2*data.(EEselection).mechEnergy(end,5) + data.(EEselection).mechEnergy(:,5)], lineColour, 'LineWidth', LineWidth);
            if plotDataSet2
                plot(time2.(EEselection), [data2.(EEselection).mechEnergy(:,5); data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5); 2*data2.(EEselection).mechEnergy(end,5) + data2.(EEselection).mechEnergy(:,5)], LineColourOpt, 'LineWidth', LineWidth);       
                legend('with interpolated points', 'original points')
            end
            if optimizeLeg.(EEselection)
                plot(time.(EEselection), [data.(EEselection).mechEnergyOpt(:,5); data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5); 2*data.(EEselection).mechEnergyOpt(end,5) + data.(EEselection).mechEnergyOpt(:,5)], lineColourOpt, 'LineWidth', LineWidth);
            end           
            grid on
            xlabel('Time [s]')
            ylabel('Energy [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title('E_{mech DFE}')
            hold off
        end
    end
       export_fig results.pdf -nocrop -append
end