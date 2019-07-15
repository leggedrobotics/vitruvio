function [] = plotEfficiencyMapWithOperatingPoints(classSelection, task)

    legCount          = classSelection.(task).basicProperties.legCount;
    linkCount         = classSelection.(task).basicProperties.linkCount;
    EEnames           = classSelection.(task).basicProperties.EEnames;
    gearRatio         = classSelection.(task).actuatorProperties.gearRatio;
    jointNames        = classSelection.(task).basicProperties.jointNames;
    actuatorSelection = classSelection.(task).actuatorProperties.actuatorSelection;
    
 %% Read in qdot and torque values for nominal and optimized designs
 for i = 1:legCount
    EEselection = EEnames(i,:);
    % Optimized qdot and torque values
    if classSelection.(task).basicProperties.optimizedLegs.(EEselection)      
        % Convert qdot and torque values from joint level to motor level.
        for j = 1:linkCount+1
            qdotMotorOpt.(EEselection)(:,j)   =  classSelection.(task).(EEselection).qdotOpt(:,j) *gearRatio.(jointNames(j,:));        
            torqueMotorOpt.(EEselection)(:,j) =  classSelection.(task).(EEselection).jointTorqueOpt(:,j)/gearRatio.(jointNames(j,:));
        end
    end
    % Nominal qdot and torque values
    for j = 1:linkCount+1 % joint count = linkCount+1
        qdotMotor.(EEselection)(:,j)   = classSelection.(task).(EEselection).qdot(:,j)*gearRatio.(jointNames(j,:));        
        torqueMotor.(EEselection)(:,j) = classSelection.(task).(EEselection).jointTorque(:,j)/gearRatio.(jointNames(j,:));
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
        markerType          = 'o';
        markerWeight = 4;
        markerColorSwing    = 'r';
        labelName1          = 'nominal swing phase';
        
        markerColorStance   = 'r'; %'[0.5430 0 0]'; % dark red
        labelName2          = 'nominal stance phase';
        
        markerColorSwingOpt = 'b';
        labelName3          = 'optimized swing phase';
        
        markerColorStanceOpt = 'b'; % dark blue 
        labelName4           = 'optimized stance phase';        

%     else % leg not optimized but the points are averaged so we have distinct swing and stance phases
%         markerTypeSwing    = 'x';
%         markerColorSwing   = 'r';
%         labelName1         = 'swing phase';
%        
%         markerTypeStance    = 'x';
%         markerColorStance   = '[0.5430 0 0]';
%         labelName2         = 'stance phase';
%     end
     
        %% plot
        numberOfContours = 10;
          for i = 1:legCount
            EEselection = EEnames(i,:);
            for j = 1:linkCount+1  
                figure('units', 'normalized','outerposition',[0 0 1 1])
                set(gcf,'color','w')
                hold on
                contourf(qdotMap.(jointNames(j,:)), torqueMap.(jointNames(j,:)), efficiencyMapCropped.(jointNames(j,:)), numberOfContours, 'ShowText', 'on', 'LabelSpacing', 800)
                plot(qdotEnvelope.(jointNames(j,:)), torqueEnvelope.(jointNames(j,:)), 'k', 'lineWidth', markerWeight)
                
                % Mirror plots to quadrant III where power is also positive
                plot(-qdotEnvelope.(jointNames(j,:)), -torqueEnvelope.(jointNames(j,:)), 'k', 'lineWidth', markerWeight)
                contourf(-qdotMap.(jointNames(j,:)), -torqueMap.(jointNames(j,:)), efficiencyMapCropped.(jointNames(j,:)), numberOfContours, 'ShowText', 'on', 'LabelSpacing', 800)

                s1 = scatter(qdotMotor.(EEselection)(:,j),torqueMotor.(EEselection)(:,j), 'filled', markerType, markerColorSwing, 'lineWidth', markerWeight, 'DisplayName', labelName1);
                %s2 = scatter(qdotMotor.(EEselection)(:,j),torqueMotor.(EEselection)(:,j), 'filled', markerType, markerColorStance, 'lineWidth', markerWeight, 'DisplayName', labelName2);                
               
%                 s1 = scatter(qdotMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwing, 'MarkerEdgeColor', markerColorSwing, 'lineWidth', 2, 'DisplayName', labelName1);
%                 s2 = scatter(qdotMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStance, 'MarkerEdgeColor', markerColorStance, 'lineWidth', 2, 'DisplayName', labelName2);                
               
                if classSelection.(task).basicProperties.optimizedLegs.(EEselection)   
                    s3 = scatter(qdotMotorOpt.(EEselection)(:,j),torqueMotorOpt.(EEselection)(:,j), 'filled', markerType, markerColorSwingOpt, 'lineWidth', markerWeight, 'DisplayName', labelName3);
                    %s4 = scatter(qdotMotorOpt.(EEselection)(:,j),torqueMotorOpt.(EEselection)(:,j), 'filled', markerType, markerColorStanceOpt, 'lineWidth', markerWeight, 'DisplayName', labelName4);                
                    
%                     s3 = scatter(qdotMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwingOpt, 'MarkerEdgeColor', markerColorSwingOpt, 'lineWidth', 2, 'DisplayName', labelName3);
%                     s4 = scatter(qdotMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStanceOpt, 'MarkerEdgeColor', markerColorStanceOpt, 'lineWidth', 2, 'DisplayName', labelName4);                
                end
                
                title(['Efficiency map for ', EEselection, ' ', jointNames(j,:)])
                xlabel('qdot [rad/s]')
                ylabel('torque [Nm]')
                
                % motion averaged, design optimized 
                if classSelection.(task).basicProperties.optimizedLegs.(EEselection)
                    legend([s1 s3])
%                 % motion averaged, design not optimized
%                 else
%                     legend([s1 s2])
                end
                xlim([-1.1*qdotMap.(jointNames(j,:))(end), 1.1*qdotMap.(jointNames(j,:))(end)])
                ylim([-1.5*torqueMap.(jointNames(j,:))(end), 1.5*torqueMap.(jointNames(j,:))(end)])
                hold off
                export_fig results.pdf -nocrop -append % Append the figure to the results pdf document

            end
          end
end