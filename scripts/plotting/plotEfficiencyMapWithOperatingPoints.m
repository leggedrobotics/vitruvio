function [] = plotEfficiencyMapWithOperatingPoints(classSelection, task, saveFiguresToPDF)

    legCount          = classSelection.(task).basicProperties.legCount;
    linkCount         = classSelection.(task).basicProperties.linkCount;
    EEnames           = classSelection.(task).basicProperties.EEnames;
    gearRatio         = classSelection.(task).actuatorProperties.gearRatio;
    jointNames        = classSelection.(task).basicProperties.jointNames;
    actuatorSelection = classSelection.(task).actuatorProperties.actuatorSelection;
    dt                = classSelection.(task).time(2) - classSelection.(task).time(1); % sample time dt is constant across the whole motion
    averageStepsForCyclicalMotion = classSelection.(task).basicProperties.trajectory.averageStepsForCyclicalMotion; % true or false statement indicating if steps were averaged or not
 %% Read in qdot and torque values for nominal and optimized designs
 for i = 1:legCount
    EEselection = EEnames(i,:);
    % Optimized qdot and torque values
    if classSelection.(task).basicProperties.optimizedLegs.(EEselection)      
        % Convert qdot and torque values from joint level to motor level.
        for j = 1:linkCount+1
            qdotMotorOpt.(EEselection)(:,j)   =  classSelection.(task).(EEselection).actuatorqdotOpt(:,j) *gearRatio.(jointNames(j,:));        
            torqueMotorOpt.(EEselection)(:,j) =  classSelection.(task).(EEselection).actuatorTorqueOpt(:,j)/gearRatio.(jointNames(j,:));
        end
    end
    % Nominal qdot and torque values
    for j = 1:linkCount+1 % joint count = linkCount+1
        qdotMotor.(EEselection)(:,j)   = classSelection.(task).(EEselection).actuatorqdot(:,j)*gearRatio.(jointNames(j,:));        
        torqueMotor.(EEselection)(:,j) = classSelection.(task).(EEselection).actuatorTorque(:,j)/gearRatio.(jointNames(j,:));
    end
    % Liftoff and touchdown timings for differentiation between swing and
    % stance phase in plot
    tLiftoff.(EEselection)   = classSelection.(task).(EEselection).tLiftoff;
    tTouchdown.(EEselection) = classSelection.(task).(EEselection).tTouchdown;
    liftoffIndex.(EEselection) = round(tLiftoff.(EEselection)/dt) + 1;
    touchdownIndex.(EEselection) = round(tTouchdown.(EEselection)/dt) + 1;
    
    % Determine which phase the leg is in at the beginning of the sampled
    % motion
    if liftoffIndex.(EEselection)(1) < touchdownIndex.(EEselection)(1)
        startingPhase.(EEselection) = 'stance';
    else
        startingPhase.(EEselection) = 'swing';
    end
    % Determine phase at end of sampled motion
    if liftoffIndex.(EEselection)(end) < touchdownIndex.(EEselection)(end)
        endingPhase.(EEselection) = 'stance';
    else
        endingPhase.(EEselection) = 'swing';
    end    
 end
 
%% Read in the values for plotting the envelope and efficiency map for each actuator and convert to motor level using gear ratio
    for i = 1:linkCount+1
        actuator = actuatorSelection.(jointNames(i,:));
        qdotEnvelope.(jointNames(i,:))         = classSelection.(task).efficiencyMap.(actuator).qdotEnvelope * gearRatio.(jointNames(i,:));
        torqueEnvelope.(jointNames(i,:))       = classSelection.(task).efficiencyMap.(actuator).torqueEnvelope / gearRatio.(jointNames(i,:));  
        qdotMap.(jointNames(i,:))              = classSelection.(task).efficiencyMap.(actuator).qdot * gearRatio.(jointNames(i,:));
        torqueMap.(jointNames(i,:))            = classSelection.(task).efficiencyMap.(actuator).torque / gearRatio.(jointNames(i,:));  
        efficiencyMapCropped.(jointNames(i,:)) = classSelection.(task).efficiencyMap.(actuator).efficiencyMapCropped;  
    end
    
%     if classSelection.trot.basicProperties.trajectory.averageStepsForCyclicalMotion  % if true, the points are averaged and we can differentiate swing and stance
        lineWidth = 2;       
        lineTypeSwing     = 'r:';        
        lineTypeStance    = 'r';        
        lineTypeSwingOpt  = 'b:'; 
        lineTypeStanceOpt = 'b'; 
     
        %% Plot torque, speed envelope and efficiency map
        numberOfContours = 10;
          for i = 1:legCount
            EEselection = EEnames(i,:);
            for j = 1:linkCount+1  
                figureName = 'Motor Efficiency with Operating Points for' + " " + EEselection + " " + jointNames(j,:);
                figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',[0 0 1 1])
                set(gcf,'color','w')
                hold on
                contourf(qdotMap.(jointNames(j,:)), torqueMap.(jointNames(j,:)), efficiencyMapCropped.(jointNames(j,:)), numberOfContours)
                plot(qdotEnvelope.(jointNames(j,:)), torqueEnvelope.(jointNames(j,:)), 'k', 'lineWidth', lineWidth)
                
                % Mirror plots to quadrant III where power is also positive
                plot(-qdotEnvelope.(jointNames(j,:)), -torqueEnvelope.(jointNames(j,:)), 'k', 'lineWidth', lineWidth)
                contourf(-qdotMap.(jointNames(j,:)), -torqueMap.(jointNames(j,:)), efficiencyMapCropped.(jointNames(j,:)), numberOfContours)
                colorbar('eastoutside')
                
                %% Plot the operating point with differentiation between swing and stance phase           
                if averageStepsForCyclicalMotion
                pNomSwing  = plot(qdotMotor.(EEselection)(1:touchdownIndex.(EEselection),j), torqueMotor.(EEselection)(1:touchdownIndex.(EEselection),j),  lineTypeSwing, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, swing');
                pNomStance = plot(qdotMotor.(EEselection)(touchdownIndex.(EEselection):end,j), torqueMotor.(EEselection)(touchdownIndex.(EEselection):end,j),  lineTypeStance, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, stance');
                    if classSelection.(task).basicProperties.optimizedLegs.(EEselection)  
                        pOptSwing  = plot(qdotMotorOpt.(EEselection)(1:touchdownIndex.(EEselection),j), torqueMotorOpt.(EEselection)(1:touchdownIndex.(EEselection),j),  lineTypeSwingOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, swing');
                        pOptStance = plot(qdotMotorOpt.(EEselection)(touchdownIndex.(EEselection)(1):end,j), torqueMotorOpt.(EEselection)(touchdownIndex.(EEselection)(1):end,j),  lineTypeStanceOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, stance');
                    end
                
                else % If the steps are not averaged and differentiating swing/stance phase is more involved
                    if strcmp(startingPhase.(EEselection), 'stance') % Leg starts in stance phase
                        plot(qdotMotor.(EEselection)(1:liftoffIndex.(EEselection)(1),j), torqueMotor.(EEselection)(1:liftoffIndex.(EEselection)(1),j),  lineTypeStance, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, stance');
                        if classSelection.(task).basicProperties.optimizedLegs.(EEselection)  
                            plot(qdotMotorOpt.(EEselection)(1:liftoffIndex.(EEselection)(1),j), torqueMotorOpt.(EEselection)(1:liftoffIndex.(EEselection)(1),j),  lineTypeStanceOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, stance');
                        end
                        for k = 1:length(liftoffIndex.(EEselection))
                            if k < min([length(liftoffIndex.(EEselection)), length(touchdownIndex.(EEselection))])
                                pNomSwing  = plot(qdotMotor.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k),j), torqueMotor.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k),j),  lineTypeSwing, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, swing');
                                pNomStance = plot(qdotMotor.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k+1),j), torqueMotor.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k+1),j),  lineTypeStance, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg stance');
                                if classSelection.(task).basicProperties.optimizedLegs.(EEselection)  
                                    pOptSwing  = plot(qdotMotorOpt.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k),j), torqueMotorOpt.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k),j),  lineTypeSwingOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, swing');
                                    pOptStance = plot(qdotMotorOpt.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k+1),j), torqueMotorOpt.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k+1),j),  lineTypeStanceOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, stance');
                                end

                            end
                        end     
                    else % Leg starts in swing phase
                        plot(qdotMotor.(EEselection)(1:touchdownIndex.(EEselection)(1),j), torqueMotor.(EEselection)(1:touchdownIndex.(EEselection)(1),j),  lineTypeSwing, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, swing');

                        for k = 1:length(touchdownIndex.(EEselection))
                            if k < min([length(liftoffIndex.(EEselection)), length(touchdownIndex.(EEselection))])
                                pNomStance = plot(qdotMotor.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k),j), torqueMotor.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k),j),  lineTypeStance, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, stance');
                                pNomSwing  = plot(qdotMotor.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k+1),j), torqueMotor.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k+1),j),  lineTypeSwing, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, swing');
                                if classSelection.(task).basicProperties.optimizedLegs.(EEselection) 
                                    pOptStance = plot(qdotMotorOpt.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k),j), torqueMotorOpt.(EEselection)(touchdownIndex.(EEselection)(k):liftoffIndex.(EEselection)(k),j),  lineTypeStanceOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, stance');
                                    pOptSwing  = plot(qdotMotorOpt.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k+1),j), torqueMotorOpt.(EEselection)(liftoffIndex.(EEselection)(k):touchdownIndex.(EEselection)(k+1),j),  lineTypeSwingOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, swing');
                                end
                            end
                        end
                    end

                    if strcmp(endingPhase,'stance')
                        plot(qdotMotor.(EEselection)(touchdownIndex.(EEselection)(end):end,j), torqueMotor.(EEselection)(touchdownIndex.(EEselection)(end):end,j),  lineTypeStance, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, stance');
                        if classSelection.(task).basicProperties.optimizedLegs.(EEselection) 
                            plot(qdotMotorOpt.(EEselection)(touchdownIndex.(EEselection)(end):end,j), torqueMotorOpt.(EEselection)(touchdownIndex.(EEselection)(end):end,j),  lineTypeStanceOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, stance');
                        end
                    else
                        plot(qdotMotor.(EEselection)(liftoffIndex.(EEselection)(end):end,j), torqueMotor.(EEselection)(liftoffIndex.(EEselection)(end):end,j),  lineTypeSwing, 'lineWidth', lineWidth, 'DisplayName', 'nominal leg, swing');                                
                        if classSelection.(task).basicProperties.optimizedLegs.(EEselection) 
                            plot(qdotMotorOpt.(EEselection)(liftoffIndex.(EEselection)(end):end,j), torqueMotorOpt.(EEselection)(liftoffIndex.(EEselection)(end):end,j),  lineTypeSwingOpt, 'lineWidth', lineWidth, 'DisplayName', 'optimized leg, swing');                                
                        end
                    end
                end
                
                title(['Efficiency map for ', EEselection, ' ', jointNames(j,:)])
                xlabel('qdot [rad/s]')
                ylabel('torque [Nm]')
                
                if classSelection.(task).basicProperties.optimizedLegs.(EEselection) 
                    legend([pNomStance pNomSwing pOptStance pOptSwing]) % Legend for optimized design
                else
                    legend([pNomStance pNomSwing]) % Legend for nominal design
                end
                xlim([-1.1*qdotMap.(jointNames(j,:))(end), 1.1*qdotMap.(jointNames(j,:))(end)])
                ylim([-1.5*torqueMap.(jointNames(j,:))(end), 1.5*torqueMap.(jointNames(j,:))(end)])
                hold off
                if saveFiguresToPDF
                    export_fig results.pdf -nocrop -append % Append the figure to the results pdf document
                end
             end
          end
end