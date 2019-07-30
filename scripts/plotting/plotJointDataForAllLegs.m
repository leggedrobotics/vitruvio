% plot joint data for all legs over 3 cycles
function [] = plotJointDataForAllLegs(data, task, data2, task2, optimizeLeg, saveFiguresToPDF)
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
    dt = data.(task).time(2) - data.(task).time(1); % sample time dt is constant across the whole motion

    % If we average the motion we string together multiple cycles
    if averageStepsForCyclicalMotion
        numberOfCycles = 3;
    else 
        numberOfCycles = 1;
    end
    
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
        maxq.(EEselection) = max(max(data.(task).(EEselection).actuatorq - mean(data.(task).(EEselection).actuatorq)));
        minq.(EEselection) = min(min(data.(task).(EEselection).actuatorq - mean(data.(task).(EEselection).actuatorq)));

        maxqdot.(EEselection) = max(max(data.(task).(EEselection).actuatorqdot));
        minqdot.(EEselection) = min(min(data.(task).(EEselection).actuatorqdot));

        maxTorque.(EEselection) = max(max(data.(task).(EEselection).actuatorTorque));
        minTorque.(EEselection) = min(min(data.(task).(EEselection).actuatorTorque));

        maxPower.(EEselection) = max(max(data.(task).(EEselection).jointPower));
        minPower.(EEselection) = min(min(data.(task).(EEselection).jointPower));

        maxMechEnergy.(EEselection) = max(max(data.(task).(EEselection).mechEnergy));
        minMechEnergy.(EEselection) = min(min(data.(task).(EEselection).mechEnergy));

        maxElecEnergy.(EEselection) = max(max(data.(task).(EEselection).elecEnergy));
        minElecEnergy.(EEselection) = min(min(data.(task).(EEselection).elecEnergy));

        % fill in the vectors for optimal limits
        qMax          = [qMax, maxq.(EEselection)];                   qMin          = [qMin, minq.(EEselection)];
        qdotMax       = [qdotMax, maxqdot.(EEselection)];             qdotMin       = [qdotMin, minqdot.(EEselection)];
        torqueMax     = [torqueMax, maxTorque.(EEselection)];         torqueMin     = [torqueMin, minTorque.(EEselection)];
        powerMax      = [powerMax, maxPower.(EEselection)];           powerMin      = [powerMin, minPower.(EEselection)];
        mechEnergyMax = [mechEnergyMax, maxMechEnergy.(EEselection)]; 
        elecEnergyMax = [elecEnergyMax, maxElecEnergy.(EEselection)]; 

        %% Get y limits for optimized design
        if optimizeLeg.(EEselection)
            maxqOpt.(EEselection) = max(max(data.(task).(EEselection).actuatorqOpt - data.(task).(EEselection).actuatorqOpt(1,:)));
            minqOpt.(EEselection) = min(min(data.(task).(EEselection).actuatorqOpt - data.(task).(EEselection).actuatorqOpt(1,:)));

            maxqdotOpt.(EEselection) = max(max(data.(task).(EEselection).actuatorqdotOpt));
            minqdotOpt.(EEselection) = min(min(data.(task).(EEselection).actuatorqdotOpt));

            maxTorqueOpt.(EEselection) = max(max(data.(task).(EEselection).actuatorTorqueOpt));
            minTorqueOpt.(EEselection) = min(min(data.(task).(EEselection).actuatorTorqueOpt));

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
            mechEnergyMaxOpt = [mechEnergyMaxOpt, maxMechEnergyOpt.(EEselection)]; 
            elecEnergyMaxOpt = [elecEnergyMaxOpt, maxElecEnergyOpt.(EEselection)]; 
        end
    end

    ylimit.actuatorq    = 1.2*[min([qMin, qMinOpt]), max([qMax, qMaxOpt])];
    ylimit.actuatorqdot = 1.2*[min([qdotMin, qdotMinOpt]), max([qdotMax, qdotMaxOpt])];
    ylimit.torque     = 1.2*[min([torqueMin, torqueMinOpt]), max([torqueMax, torqueMaxOpt])];
    ylimit.power      = 1.2*[min([powerMin, powerMinOpt]), max([powerMax, powerMaxOpt])];
    ylimit.mechEnergy = 1.2*[0, max([mechEnergyMax, mechEnergyMaxOpt])];

     if averageStepsForCyclicalMotion % When the motion is averaged we repeat the motion 3 times.
        ylimit.mechEnergy = [0, 3*max([mechEnergyMax mechEnergyMaxOpt])]; 
        ylimit.elecEnergy = [0, 3*max([elecEnergyMax  elecEnergyMaxOpt])];
    else % When the motion is not averaged we should the sampled range without repetition.
        ylimit.mechEnergy = [0, max([mechEnergyMax mechEnergyMaxOpt])]; 
        ylimit.elecEnergy = [0, max([elecEnergyMax  elecEnergyMaxOpt])];    
    end

    %% Create patches to represent swing/stance phase
    % Light blue patch behind stance phase, swing phase left white.
    patchColor = [0.5843 0.8157 0.9882]; % Light blue
    patchAlpha = 0.5; % Transparency
    
    Y_qTemp      = [ylimit.actuatorq(1); ylimit.actuatorq(2); ylimit.actuatorq(2); ylimit.actuatorq(1)];
    Y_qdotTemp   = [ylimit.actuatorqdot(1); ylimit.actuatorqdot(2); ylimit.actuatorqdot(2); ylimit.actuatorqdot(1)];
    Y_torqueTemp = [ylimit.torque(1); ylimit.torque(2); ylimit.torque(2); ylimit.torque(1)];
    Y_powerTemp  = [ylimit.power(1); ylimit.power(2); ylimit.power(2); ylimit.power(1)];
    Y_mechEnergyTemp  = [ylimit.mechEnergy(1); ylimit.mechEnergy(2); ylimit.mechEnergy(2); ylimit.mechEnergy(1)];

    for i = 1:legCount
        EEselection = EEnames(i,:);
        tLiftoff.(EEselection)   = data.(task).(EEselection).tLiftoff;
        tTouchdown.(EEselection) = data.(task).(EEselection).tTouchdown;
        
        if averageStepsForCyclicalMotion
            timePerCycle = length(data.(task).(EEselection).actuatorq(:,1))*dt;
            stanceTime = tLiftoff.(EEselection)(2) - tLiftoff.(EEselection)(1);
            % Steps are averaged such that liftoff occurs at timestep 1
            for j = 1:numberOfCycles
                X.(EEselection)(:,j) = [tTouchdown.(EEselection)(1); tTouchdown.(EEselection)(1); stanceTime; stanceTime];
                X.(EEselection)(:,j) = X.(EEselection)(:,j) + timePerCycle*(j-1);
            end
        else
            % Determine which phase the leg is in at the beginning of the sampled
            % motion
            if tLiftoff.(EEselection)(1) < tTouchdown.(EEselection)(1)
                startingPhase.(EEselection) = 'stance';
            else
                startingPhase.(EEselection) = 'swing';
            end
            % Determine phase at end of sampled motion
            if tLiftoff.(EEselection)(end) < tTouchdown.(EEselection)(end)
                endingPhase.(EEselection) = 'stance';
            else
                endingPhase.(EEselection) = 'swing';
            end    

            if strcmp(startingPhase.(EEselection), 'stance') % Leg starts in stance
                X.(EEselection)(1,1) = 0;
                X.(EEselection)(2,1) = 0;
                X.(EEselection)(3,1) = tLiftoff.(EEselection)(1);
                X.(EEselection)(4,1) = tLiftoff.(EEselection)(1);
                for j = 1:length(tLiftoff.(EEselection))-1 
                    X.(EEselection)(1,j+1) = tTouchdown.(EEselection)(j);
                    X.(EEselection)(2,j+1) = tTouchdown.(EEselection)(j);
                    X.(EEselection)(3,j+1) = tLiftoff.(EEselection)(j+1);
                    X.(EEselection)(4,j+1) = tLiftoff.(EEselection)(j+1);
                end
            else % Leg starts in swing
                for j = 1:min([length(tLiftoff.(EEselection)), length(tTouchdown.(EEselection))])
                    X.(EEselection)(1,j) = tTouchdown.(EEselection)(j);
                    X.(EEselection)(2,j) = tTouchdown.(EEselection)(j);
                    X.(EEselection)(3,j) = tLiftoff.(EEselection)(j);
                    X.(EEselection)(4,j) = tLiftoff.(EEselection)(j);
                end
            end

            if strcmp(endingPhase.(EEselection), 'stance')
                X.(EEselection)(1,j+2) = tTouchdown.(EEselection)(end);
                X.(EEselection)(2,j+2) = tTouchdown.(EEselection)(end);    
                X.(EEselection)(3,j+2) = length(data.(task).(EEselection).actuatorq); % Continue last patch until end of plotted data
                X.(EEselection)(4,j+2) = length(data.(task).(EEselection).actuatorq);
            end
            % Shift the points to the starting time of the sampled motion
            X.(EEselection) = X.(EEselection) + data.(task).time(startIndexFullTrajectory);
        end
        Y_q.(EEselection)      = repmat(Y_qTemp, 1, length(X.(EEselection)(1,:)));
        Y_qdot.(EEselection)   = repmat(Y_qdotTemp, 1, length(X.(EEselection)(1,:)));
        Y_torque.(EEselection) = repmat(Y_torqueTemp, 1, length(X.(EEselection)(1,:)));
        Y_power.(EEselection)  = repmat(Y_powerTemp, 1, length(X.(EEselection)(1,:)));
        Y_mechEnergy.(EEselection)  = repmat(Y_mechEnergyTemp, 1, length(X.(EEselection)(1,:)));        
    end
    
     %% Repeat data multiple times if we average one cycle to string together multiple steps.
        for i = 1:legCount
            EEselection = EEnames(i,:);
            for j = 1:linkCount+1
                q.(EEselection)(:,j)           = repmat(data.(task).(EEselection).actuatorq(:,j), numberOfCycles, 1);
                qdot.(EEselection)(:,j)        = repmat(data.(task).(EEselection).actuatorqdot(:,j), numberOfCycles, 1);
                actuatorTorque.(EEselection)(:,j) = repmat(data.(task).(EEselection).actuatorTorque(:,j), numberOfCycles, 1);
                activeTorque.(EEselection)(:,j) = repmat(data.(task).(EEselection).activeTorque(:,j), numberOfCycles, 1);
                passiveTorque.(EEselection)(:,j) = repmat(data.(task).(EEselection).passiveTorque(:,j), numberOfCycles, 1);
                jointTorque.(EEselection)(:,j) = repmat(data.(task).(EEselection).jointTorque(:,j), numberOfCycles, 1);                
                jointPower.(EEselection)(:,j)  = repmat(data.(task).(EEselection).jointPower(:,j), numberOfCycles, 1);
                mechEnergy.(EEselection)(:,j)  = repmat(data.(task).(EEselection).mechEnergy(:,j), numberOfCycles, 1);
                if optimizeLeg.(EEselection)
                    qOpt.(EEselection)(:,j)           = repmat(data.(task).(EEselection).actuatorqOpt(:,j), numberOfCycles, 1);
                    qdotOpt.(EEselection)(:,j)        = repmat(data.(task).(EEselection).actuatorqdotOpt(:,j), numberOfCycles, 1);
                    actuatorTorqueOpt.(EEselection)(:,j) = repmat(data.(task).(EEselection).actuatorTorqueOpt(:,j), numberOfCycles, 1);
                    jointPowerOpt.(EEselection)(:,j)  = repmat(data.(task).(EEselection).jointPowerOpt(:,j), numberOfCycles, 1);
                    mechEnergyOpt.(EEselection)(:,j)  = repmat(data.(task).(EEselection).mechEnergyOpt(:,j), numberOfCycles, 1);                
                end
                if plotDataSet2
                    q2.(EEselection)(:,j)           = repmat(data2.(task2).(EEselection).actuatorq(:,j), numberOfCycles, 1);
                    qdot2.(EEselection)(:,j)        = repmat(data2.(task2).(EEselection).actuatorqdot(:,j), numberOfCycles, 1);
                    actuatorTorque2.(EEselection)(:,j) = repmat(data2.(task2).(EEselection).actuatorTorque(:,j), numberOfCycles, 1);
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
        % one cycle repeated multiple times
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

    %% Actuator Position
    % Shift angles to start at 0 rad.
    for i = 1:legCount
        EEselection = EEnames(i,:);        
        q.(EEselection) = q.(EEselection) - mean(q.(EEselection)); % normalize so first point at zero
        if plotDataSet2    
            q2.(EEselection) = q2.(EEselection) - mean(q2.(EEselection)); % normalize so first point at zero
        end
        if optimizeLeg.(EEselection)
            qOpt.(EEselection) = qOpt.(EEselection)(:,:) - mean(qOpt.(EEselection)); % normalize so first point at zero
        end
    end
    
    figure('name', 'Actuator Position', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'q_{HAA}','q_{HFE}','q_{KFE}','q_{AFE}','q_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            patch(X.(EEselection), Y_q.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(q.(EEselection))-1),  q.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(q2.(EEselection))-1),  q2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(qOpt.(EEselection))-1),  qOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 
                    legend([p(1) p(2)],'approximated inertia', 'exact inertia')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3)],'nominal','optimized')
                end
            end            
            
            grid on
            xlabel('Time [s]')
            ylabel('Position [rad]')
            xlim(xlimit.time)
            ylim(ylimit.actuatorq)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end

    %% Actuator Velocity
    figure('name', 'Actuator Velocity', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'\omega_{HAA}','\omega_{HFE}','\omega_{KFE}','\omega_{AFE}','\omega_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            patch(X.(EEselection), Y_qdot.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(qdot.(EEselection))-1),  qdot.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(qdot2.(EEselection))-1),  qdot2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(qdotOpt.(EEselection))-1),  qdotOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            p(5) = line([min(xlim),max(xlim)],[-data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)), -data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');            
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 && max(ylimit.actuatorqdot) > data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)) % If actuator limits visible on plot
                    legend([p(1) p(2) p(4)], 'approximated inertia', 'exact inertia', 'actuator limits')
                elseif plotDataSet2 % Actuator limits not visible
                    legend([p(1) p(2)], 'approximated inertia', 'exact inertia')                    
                elseif optimizeLeg.(EEselection) && max(ylimit.actuatorqdot) > data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:)) 
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3)],'nominal','optimized')
                elseif max(ylimit.actuatorqdot) > data.(task).actuatorProperties.maxqdotLimit.(jointNames(j,:))
                    legend(p(4), 'actuator limits')
                end
            end
                
            grid on
            xlabel('Time [s]')
            ylabel('Velocity [rad/s]')
            xlim(xlimit.time)
            ylim(ylimit.actuatorqdot)
            title([EEselection, ' ', plotTitle{k}])
            k = k+1;
            hold off
        end
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    
    %% Actuator Torque
    figure('name', 'Actuator Torque', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'\tau_{HAA}','\tau_{HFE}','\tau_{KFE}','\tau_{AFE}','\tau_{DFE}'};
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            patch(X.(EEselection), Y_torque.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(actuatorTorque.(EEselection))-1),  actuatorTorque.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(actuatorTorque2.(EEselection))-1),  actuatorTorque2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(actuatorTorqueOpt.(EEselection))-1),  actuatorTorqueOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            p(5) = line([min(xlim),max(xlim)],[-data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), -data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');               
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 && max(ylimit.torque) > data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)) % If actuator limits visible on plot
                    legend([p(1) p(2) p(4)], 'approximated inertia', 'exact inertia', 'actuator limits')
                elseif plotDataSet2 % Actuator limits not visible
                    legend([p(1) p(2)], 'approximated inertia', 'exact inertia')                    
                elseif optimizeLeg.(EEselection) && max(ylimit.torque) > data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)) 
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3)],'nominal','optimized')
                elseif max(ylimit.torque) > data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))
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
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    
    %% Active/Passive Torque
    figure('name', 'Active/Passive Torque', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'\tau_{HAA}','\tau_{HFE}','\tau_{KFE}','\tau_{AFE}','\tau_{DFE}'};
    activeTorqueColor  = 'r';
    passiveTorqueColor = 'b';
    jointTorqueColor   = 'k';
    
    activeTorqueColorOpt  = 'r--';
    passiveTorqueColorOpt = 'b--';
    jointTorqueColorOpt   = 'k--';
    
    for i = 1:legCount
        k = 1; % plot title index
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            patch(X.(EEselection), Y_torque.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(activeTorque.(EEselection))-1),  activeTorque.(EEselection)(:,j), activeTorqueColor, 'LineWidth', LineWidth);
            p(2) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(passiveTorque.(EEselection))-1),  passiveTorque.(EEselection)(:,j), passiveTorqueColor, 'LineWidth', LineWidth);
            p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointTorque.(EEselection))-1),  jointTorque.(EEselection)(:,j), jointTorqueColor, 'LineWidth', LineWidth);            
            if plotDataSet2    
                p(4) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(activeTorque.(EEselection))-1),  activeTorque.(EEselection)(:,j), activeTorqueColor, 'LineWidth', LineWidth);
                p(5) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(passiveTorque.(EEselection))-1),  passiveTorque.(EEselection)(:,j), passiveTorqueColor, 'LineWidth', LineWidth);
                p(6) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(jointTorque.(EEselection))-1),  jointTorque.(EEselection)(:,j), jointTorqueColor, 'LineWidth', LineWidth);            
            end
            if optimizeLeg.(EEselection)
                p(7) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(activeTorqueOpt.(EEselection))-1),  activeTorqueOpt.(EEselection)(:,j), activeTorqueColorOpt, 'LineWidth', LineWidth);
                p(8) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(passiveTorqueOpt.(EEselection))-1),  passiveTorqueOpt.(EEselection)(:,j), passiveTorqueColorOpt, 'LineWidth', LineWidth);
                p(9) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointTorqueOpt.(EEselection))-1),  jointTorqueOpt.(EEselection)(:,j), jointTorqueColorOpt, 'LineWidth', LineWidth);            
            end
            % plot actuator limits on the same plot
            p(10) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            p(11) = line([min(xlim),max(xlim)],[-data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)), -data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');               
            
            if j == 1 % only show legend on HAA subplots
                if ~optimizeLeg.(EEselection) && max(ylimit.torque) > data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)) % If actuator limits visible on plot
                    legend([p(1) p(2) p(3) p(10)], 'active torque', 'passive torque', 'joint torque', 'actuator limits')
                elseif ~optimizeLeg.(EEselection) && max(ylimit.torque) < data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)) % Actuator limits not visible
                    legend([p(1) p(2) p(3)], 'active torque', 'passive torque', 'joint torque')
                elseif optimizeLeg.(EEselection) && max(ylimit.torque) > data.(task).actuatorProperties.maxTorqueLimit.(jointNames(j,:)) 
                    legend([p(1) p(2) p(3) p(7) p(8) p(9) p(10)], 'active torque', 'passive torque', 'joint torque', 'active torque opt', 'passive torque opt', 'joint torque opt', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(2) p(3) p(7) p(8) p(9)], 'active torque', 'passive torque', 'joint torque', 'active torque opt', 'passive torque opt', 'joint torque opt')
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
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    
    
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
            patch(X.(EEselection), Y_power.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(jointPower.(EEselection))-1),  jointPower.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(actuatorTorque2.(EEselection))-1),  jointPower2.(EEselection)(:,j), 'LineWidth', LineWidth);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(actuatorTorqueOpt.(EEselection))-1),  jointPowerOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            % plot actuator limits on the same plot
            p(4) = line([min(xlim),max(xlim)],[data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:)), data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:))], 'Color', 'k', 'LineStyle', '--');
            grid on
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 && max(ylimit.power) > data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:)) % If actuator limits visible on plot
                    legend([p(1) p(2) p(4)], 'approximated inertia', 'exact inertia', 'actuator limits')
                elseif plotDataSet2 % Actuator limits not visible
                    legend([p(1) p(2)], 'approximated inertia', 'exact inertia')                    
                elseif optimizeLeg.(EEselection) && max(ylimit.power) > data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:)) 
                    legend([p(1) p(3) p(4)],'nominal','optimized', 'actuator limits')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3)],'nominal','optimized')
                elseif max(ylimit.power) > data.(task).actuatorProperties.maxPowerLimit.(jointNames(j,:))
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
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    
    %% Joint Energy
    figure('name', 'Joint Energy Consumption', 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
    set(gcf,'color','w')
    plotTitle = {'E_{mech HAA}','E_{mech HFE}','E_{mech KFE}','E_{mech AFE}','E_{mech DFE}'};
    for i = 1:legCount
        k = 1; % Index for plot title
        EEselection = EEnames(i,:);
        for j = 1:linkCount+1
            subplot(linkCount+1, legCount, i + (j-1)*legCount);
            hold on
            patch(X.(EEselection), Y_mechEnergy.(EEselection), patchColor, 'FaceAlpha', patchAlpha, 'EdgeAlpha', 0)            
            p(1) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(mechEnergy.(EEselection))-1),  mechEnergy.(EEselection)(:,j), lineColour, 'LineWidth', LineWidth);
            if plotDataSet2    
                p(2) = plot(data2.(task2).time(startTimeIndex:startTimeIndex+length(mechEnergy2.(EEselection))-1),  mechEnergy2.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            if optimizeLeg.(EEselection)
                p(3) = plot(data.(task).time(startTimeIndex:startTimeIndex+length(mechEnergyOpt.(EEselection))-1),  mechEnergyOpt.(EEselection)(:,j), lineColourOpt, 'LineWidth', LineWidth);
            end
            
            if j == 1 % only show legend on HAA subplots
                if plotDataSet2 
                    legend([p(1) p(2)], 'approximated inertia', 'exact inertia')
                elseif optimizeLeg.(EEselection)
                    legend([p(1) p(3)],'nominal','optimized')
                end
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
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
end