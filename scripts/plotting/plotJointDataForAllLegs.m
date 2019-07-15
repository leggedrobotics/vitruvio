% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(data, task, data2, task2, optimizeLeg)

    legCount                      = data.(task).basicProperties.legCount;
    linkCount                     = data.(task).basicProperties.linkCount;
    EEnames                       = data.(task).basicProperties.EEnames;
    jointNames                    = data.(task).basicProperties.jointNames;
    averageStepsForCyclicalMotion = data.(task).basicProperties.trajectory.averageStepsForCyclicalMotion;
    removalRatioStart             = data.(task).basicProperties.trajectory.removalRatioStart;
    removalRatioEnd               = data.(task).basicProperties.trajectory.removalRatioEnd;
    startIndexFullTrajectory      = round(length(data.(task).fullTrajectory.r.EEdes.LF)*removalRatioStart);
    startIndexFullTrajectory(startIndexFullTrajectory<1) = 1;
    endIndexFullTrajectory        = round(length(data.(task).fullTrajectory.r.EEdes.LF)*(1-removalRatioEnd));

    %% Second data set
    if isempty(data2)
         plotDataSet2 = false;
    else
         plotDataSet2 = true;
         dt2 = data2.(task2).time(2) - data2.(task2).time(1);
    end

    %% Load in data and plot options
    dt = data.(task).time(2) - data.(task).time(1); % time is uniform so dt is constant

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

        % Return max and min values for nominal design
        maxq.(EEselection) = max(max(data.(task).(EEselection).q - data.(task).(EEselection).q(1,:)));
        minq.(EEselection) = min(min(data.(task).(EEselection).q - data.(task).(EEselection).q(1,:)));

        maxqdot.(EEselection) = max(max(data.(task).(EEselection).qdot));
        minqdot.(EEselection) = min(min(data.(task).(EEselection).qdot));

        maxTorque.(EEselection) = max(max(data.(task).(EEselection).jointTorque));
        minTorque.(EEselection) = min(min(data.(task).(EEselection).jointTorque));

        maxPower.(EEselection) = max(max(data.(task).(EEselection).jointPower));
        minPower.(EEselection) = min(min(data.(task).(EEselection).jointPower));

        maxMechEnergy.(EEselection) = max(max(data.(task).(EEselection).mechEnergy));
        minMechEnergy.(EEselection) = min(min(data.(task).(EEselection).mechEnergy));

        maxElecEnergy.(EEselection) = max(max(data.(task).(EEselection).elecEnergy));
        minElecEnergy.(EEselection) = min(min(data.(task).(EEselection).elecEnergy));

        % fill in the vectors for optimal limits
        qMax          = [qMax, maxq.(EEselection)];                            qMin = [qMin, minq.(EEselection)];
        qdotMax       = [qdotMax, maxqdot.(EEselection)];                   qdotMin = [qdotMin, minqdot.(EEselection)];
        torqueMax     = [torqueMax, maxTorque.(EEselection)];             torqueMin = [torqueMin, minTorque.(EEselection)];
        powerMax      = [powerMax, maxPower.(EEselection)];                powerMin = [powerMin, minPower.(EEselection)];
        mechEnergyMax = [mechEnergyMax, maxMechEnergy.(EEselection)]; mechEnergyMin = [mechEnergyMin, minMechEnergy.(EEselection)];
        elecEnergyMax = [elecEnergyMax, maxElecEnergy.(EEselection)]; elecEnergyMin = [elecEnergyMin, minElecEnergy.(EEselection)];

        %% Get y limits for optimized design
        if optimizeLeg.(EEselection)
            maxqOpt.(EEselection) = max(max(data.(task).(EEselection).qOpt - data.(task).(EEselection).qOpt(1,:)));
            minqOpt.(EEselection) = min(min(data.(task).(EEselection).qOpt - data.(task).(EEselection).qOpt(1,:)));

            maxqdotOpt.(EEselection) = max(max(data.(task).(EEselection).qdotOpt));
            minqdotOpt.(EEselection) = min(min(data.(task).(EEselection).qdotOpt));

            maxTorqueOpt.(EEselection) = max(max(data.(task).(EEselection).jointTorqueOpt));
            minTorqueOpt.(EEselection) = min(min(data.(task).(EEselection).jointTorqueOpt));

            maxPowerOpt.(EEselection) = max(max(data.(task).(EEselection).jointPowerOpt));
            minPowerOpt.(EEselection) = min(min(data.(task).(EEselection).jointPowerOpt));

            maxMechEnergyOpt.(EEselection) = max(max(data.(task).(EEselection).mechEnergyOpt));
            minMechEnergyOpt.(EEselection) = min(min(data.(task).(EEselection).mechEnergyOpt));

            maxElecEnergyOpt.(EEselection) = max(max(data.(task).(EEselection).elecEnergyOpt));
            minElecEnergyOpt.(EEselection) = min(min(data.(task).(EEselection).elecEnergyOpt));

            % fill in the vectors for nominal limits
            qMaxOpt          = [qMaxOpt, maxqOpt.(EEselection)];                   qMin             = [qMinOpt, minqOpt.(EEselection)];
            qdotMaxOpt       = [qdotMaxOpt, maxqdotOpt.(EEselection)];             qdotMinOpt       = [qdotMinOpt, minqdotOpt.(EEselection)];
            torqueMaxOpt     = [torqueMaxOpt, maxTorqueOpt.(EEselection)];         torqueMinOpt     = [torqueMinOpt, minTorqueOpt.(EEselection)];
            powerMaxOpt      = [powerMaxOpt, maxPowerOpt.(EEselection)];           powerMinOpt      = [powerMinOpt, minPowerOpt.(EEselection)];
            mechEnergyMaxOpt = [mechEnergyMaxOpt, maxMechEnergyOpt.(EEselection)]; mechEnergyMinOpt = [mechEnergyMinOpt, minMechEnergyOpt.(EEselection)];
            elecEnergyMaxOpt = [elecEnergyMaxOpt, maxElecEnergyOpt.(EEselection)]; elecEnergyMinOpt = [elecEnergyMinOpt, minElecEnergyOpt.(EEselection)];
        end
    end

    ylimit.q          = [min([qMin, qMinOpt]), max([qMax, qMaxOpt])]                     + [-0.2*abs(min(qMin)), 0.2*abs(max(qMax))];
    ylimit.qdot       = [min([qdotMin, qdotMinOpt]), max([qdotMax, qdotMaxOpt])]         + [-0.2*abs(min(qdotMin)), 0.2*abs(max(qdotMax))];
    ylimit.torque     = [min([torqueMin, torqueMinOpt]), max([torqueMax, torqueMaxOpt])] + [-0.2*abs(min(torqueMin)), 0.2*abs(max(torqueMax))];
    ylimit.power      = [min([powerMin, powerMinOpt]), max([powerMax, powerMaxOpt])]     + [-0.2*abs(min(powerMin)), 0.2*abs(max(powerMax))];

    if averageStepsForCyclicalMotion % When the motion is averaged we repeat the motion 3 times.
        ylimit.mechEnergy = [0, 3*max([mechEnergyMax mechEnergyMaxOpt])]; 
        ylimit.elecEnergy = [0, 3*max([elecEnergyMax  elecEnergyMaxOpt])];
    else % When the motion is not averaged we should the sampled range without repetition.
        ylimit.mechEnergy = [0, max([mechEnergyMax mechEnergyMaxOpt])]; 
        ylimit.elecEnergy = [0, max([elecEnergyMax  elecEnergyMaxOpt])];    
    end

    %% Repeat data multiple times if we average one cycle to string together multiple steps.
    if averageStepsForCyclicalMotion
        numberOfCycles = 3;
    else 
        numberOfCycles = 1;
    end
        for i = 1:legCount
            EEselection = EEnames(i,:);
            for j = 1:linkCount+1
                q.(EEselection)(:,j)           = repmat(data.(task).(EEselection).q(:,j), numberOfCycles, 1);
                qdot.(EEselection)(:,j)        = repmat(data.(task).(EEselection).qdot(:,j), numberOfCycles, 1);
                jointTorque.(EEselection)(:,j) = repmat(data.(task).(EEselection).jointTorque(:,j), numberOfCycles, 1);
                jointPower.(EEselection)(:,j)  = repmat(data.(task).(EEselection).jointPower(:,j), numberOfCycles, 1);
                mechEnergy.(EEselection)(:,j)  = repmat(data.(task).(EEselection).mechEnergy(:,j), numberOfCycles, 1);
                if optimizeLeg.(EEselection)
                    qOpt.(EEselection)(:,j)           = repmat(data.(task).(EEselection).qOpt(:,j), numberOfCycles, 1);
                    qdotOpt.(EEselection)(:,j)        = repmat(data.(task).(EEselection).qdotOpt(:,j), numberOfCycles, 1);
                    jointTorqueOpt.(EEselection)(:,j) = repmat(data.(task).(EEselection).jointTorqueOpt(:,j), numberOfCycles, 1);
                    jointPowerOpt.(EEselection)(:,j)  = repmat(data.(task).(EEselection).jointPowerOpt(:,j), numberOfCycles, 1);
                    mechEnergyOpt.(EEselection)(:,j)  = repmat(data.(task).(EEselection).mechEnergyOpt(:,j), numberOfCycles, 1);                
                end
                if plotDataSet2
                    q2.(EEselection)(:,j)           = repmat(data2.(task2).(EEselection).q(:,j), numberOfCycles, 1);
                    qdot2.(EEselection)(:,j)        = repmat(data2.(task2).(EEselection).qdot(:,j), numberOfCycles, 1);
                    jointTorque2.(EEselection)(:,j) = repmat(data2.(task2).(EEselection).jointTorque(:,j), numberOfCycles, 1);
                    jointPower2.(EEselection)(:,j)  = repmat(data2.(task2).(EEselection).jointPower(:,j), numberOfCycles, 1);
                    mechEnergy2.(EEselection)(:,j)  = repmat(data2.(task2).(EEselection).mechEnergy(:,j), numberOfCycles, 1);
                end
            end  

            if averageStepsForCyclicalMotion
                % Energy adds onto last value when cycling
                mechEnergy.(EEselection) = data.(task).(EEselection).mechEnergy;
                if optimizeLeg.(EEselection)
                    mechEnergyOpt.(EEselection) = data.(task).(EEselection).mechEnergyOpt;
                end
                if plotDataSet2
                    mechEnergy2.(EEselection) = data2.(task2).(EEselection).mechEnergy;
                end
                for k = 2:numberOfCycles
                    mechEnergy.(EEselection) = [mechEnergy.(EEselection);  mechEnergy.(EEselection)(end,:) + data.(task).(EEselection).mechEnergy]; 
                    if optimizeLeg.(EEselection)
                        mechEnergyOpt.(EEselection) = [mechEnergyOpt.(EEselection);  mechEnergyOpt.(EEselection)(end,:) + data.(task).(EEselection).mechEnergyOpt]; 
                    end
                    if plotDataSet2
                        mechEnergy2.(EEselection) = [mechEnergy2.(EEselection);  mechEnergy2.(EEselection)(end,:) + data2.(task).(EEselection).mechEnergy]; 
                    end                
                end
            end
        end

    if averageStepsForCyclicalMotion
        % one cycle repeated three times
        xlimit.time = [0, dt*numberOfCycles*length(data.(task).LF.r.EE)];  
    else
        % sampled motion range
        xlimit.time = [data.(task).time(startIndexFullTrajectory), data.(task).time(endIndexFullTrajectory)]; 
    end

    %% Plot settings
    lineColour = 'r';
    faceColour = 'r';
    lineColourOpt = 'b';
    faceColourOpt = 'b';

    LineWidth = 1;
    scatterSize = 12;

    if averageStepsForCyclicalMotion
        startTimeIndex = 1;
    else 
        startTimeIndex = startIndexFullTrajectory;
    end

    %% Joint Position
    % Shift angles to start at 0 rad.
    for i = 1:legCount
        EEselection = EEnames(i,:);        
        q.(EEselection) = q.(EEselection)(:,:) - q.(EEselection)(1,:); % normalize so first point at zero
        if optimizeLeg.(EEselection)
            qOpt.(EEselection) = qOpt.(EEselection)(:,:) - qOpt.(EEselection)(1,:); % normalize so first point at zero
        end
    end
    
    figure('name', 'Joint Position', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'q_{HAA}','q_{HFE}','q_{KFE}','q_{AFE}','q_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            plot(data.(task).time(startTimeIndex:startTimeIndex+length(q.(EEselection))-1),  q.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                plot(data2.(task).time(startTimeIndex:startTimeIndex+length(q.(EEselection))-1),  q2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')  
            end
            if optimizeLeg.(EEselection)
                plot(data.(task).time(startTimeIndex:startTimeIndex+length(qOpt.(EEselection))-1),  qOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
                legend('nominal', 'optimized')
            end
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.q)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    export_fig results.pdf -nocrop -append

    %% Joint Velocity
    figure('name', 'Joint Velocity', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'\omega_{HAA}','\omega_{HFE}','\omega_{KFE}','\omega_{AFE}','\omega_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(qdot.(EEselection))-1),  qdot.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task).time(startTimeIndex:startTimeIndex+length(qdot2.(EEselection))-1),  qdot2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(qdotOpt.(EEselection))-1),  qdotOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            p(5) = line([min(xlim),max(xlim)],[-data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)), -data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');            
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 
                    legend([p(1) p(2) p(4)],'nominal data','data set 2', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                else
                    legend(p(4), 'actuator limits')
                end
            end
                
            grid on
            xlabel('Time [s]')
            ylabel('Velocity [rad/s]')
            xlim(xlimit.time)
            ylim(ylimit.qdot)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    export_fig results.pdf -nocrop -append
    
    %% Joint Torque
    figure('name', 'Joint Torque', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'\tau_{HAA}','\tau_{HFE}','\tau_{KFE}','\tau_{AFE}','\tau_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointTorque.(EEselection))-1),  jointTorque.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(jointTorque2.(EEselection))-1),  jointTorque2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointTorqueOpt.(EEselection))-1),  jointTorqueOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            p(5) = line([min(xlim),max(xlim)],[-data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), -data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');               
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 
                    legend([p(1) p(2) p(4)],'nominal data','data set 2', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                else
                    legend(p(4), 'actuator limits')
                end
            end
            
            grid on
            xlabel('Time [s]')
            ylabel('Torque [Nm]')
            xlim(xlimit.time)
            ylim(ylimit.torque)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    export_fig results.pdf -nocrop -append
  
    %% Joint Power
    figure('name', 'Joint Power', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'P_{mech HAA}','P_{mech HFE}','P_{mech KFE}','P_{mech AFE}','P_{mech DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointPower.(EEselection))-1),  jointPower.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(jointTorque2.(EEselection))-1),  jointPower2.(EEselection)(:,j), 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointTorqueOpt.(EEselection))-1),  jointPowerOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            grid on
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 
                    legend([p(1) p(2) p(4)],'nominal data','data set 2', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                else
                    legend(p(4), 'actuator limits')
                end
            end
            
            xlabel('Time [s]')
            ylabel('P_{mech} [W]')
            xlim(xlimit.time)
            ylim(ylimit.power)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    export_fig results.pdf -nocrop -append
   
    %% Joint Energy
    figure('name', 'Joint Energy Consumption', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'E_{mech HAA}','E_{mech HFE}','E_{mech KFE}','E_{mech AFE}','E_{mech DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            plot(data.(task).time(startTimeIndex:startTimeIndex+length(mechEnergy.(EEselection))-1),  mechEnergy.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(mechEnergy2.(EEselection))-1),  mechEnergy2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth,'MarkerFaceColor', faceColourOpt);
                legend('with interpolated points', 'original points')  
            end
            if optimizeLeg.(EEselection)
                plot(data.(task).time(startTimeIndex:startTimeIndex+length(mechEnergyOpt.(EEselection))-1),  mechEnergyOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
                legend('nominal', 'optimized')
            end
            grid on
            xlabel('Time [s]')
            ylabel('E_{mech} [J]')
            xlim(xlimit.time)
            ylim(ylimit.mechEnergy)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    export_fig results.pdf -nocrop -append
end    