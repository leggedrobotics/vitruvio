function [] = plotEfficiencyMapWithOperatingPoints(classSelection, task)

    legCount          = classSelection.(task).basicProperties.legCount;
    linkCount         = classSelection.(task).basicProperties.linkCount;
    EEnames           = classSelection.(task).basicProperties.EEnames;
    gearRatio         = classSelection.(task).actuatorProperties.gearRatio;
    jointNames        = classSelection.(task).basicProperties.jointNames;
    actuatorSelection = classSelection.(task).actuatorProperties.actuatorSelection;
    
 for i = 1:legCount
    EEselection = EEnames(i,:);

    %% Optimized qdot and torque values
    if classSelection.(task).basicProperties.optimizedLegs.(EEselection)      
        % Convert qdot and torque values from joint level to motor level.
        for i = 1:linkCount+1
            qdotMotorOpt.(EEselection)(:,i)   =  classSelection.(task).(EEselection).qdotOpt(:,i) *gearRatio.(jointNames(i,:));        
            torqueMotorOpt.(EEselection)(:,i) =  classSelection.(task).(EEselection).jointTorqueOpt(:,i)/gearRatio.(jointNames(i,:));
        end
    end

    %% Nominal qdot and torque values
    for i = 1:linkCount+1 % joint count = linkCount+1
        qdotMotor.(EEselection)(:,i)   = classSelection.(task).(EEselection).qdot(:,i)*gearRatio.(jointNames(i,:));        
        torqueMotor.(EEselection)(:,i) = classSelection.(task).(EEselection).jointTorque(:,i)/gearRatio.(jointNames(i,:));
    end
 end
 
%% Read in the values for plotting the envelope and efficiency map
    for i = 1:linkCount+1
        actuator = actuatorSelection.(jointNames(i,:));
        qdotEnvelope.(jointNames(i,:))         = classSelection.(task).efficiencyMap.(actuator).qdotEnvelope;
        torqueEnvelope.(jointNames(i,:))       = classSelection.(task).efficiencyMap.(actuator).torqueEnvelope;  
        qdotMap.(jointNames(i,:))              = classSelection.(task).efficiencyMap.(actuator).qdot;
        torqueMap.(jointNames(i,:))            = classSelection.(task).efficiencyMap.(actuator).torque;  
        efficiencyMapCropped.(jointNames(i,:)) = classSelection.(task).efficiencyMap.(actuator).efficiencyMapCropped;  

    end
    
%     if classSelection.trot.basicProperties.trajectory.averageStepsForCyclicalMotion  % if true, the points are averaged and we can differentiate swing and stance
        markerType          = 'x';
        markerColorSwing    = 'r';
        labelName1          = 'nominal swing phase';
        
        markerColorStance   = '[0.5430 0 0]'; % dark red
        labelName2          = 'nominal stance phase';
        
        markerColorSwingOpt = 'b';
        labelName3          = 'optimized swing phase';
        
        markerColorStanceOpt = '[0 0 0.5430]'; % dark blue 
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
        figure('name', 'Actuator efficiency map with operating points', 'DefaultAxesFontSize', 10)
        jointNames = {'HAA'; 'HFE'; 'KFE'; 'AFE'; 'DFE'};
          for j = 1:legCount
            EEselection = EEnames(j,:);
            for i = 1:linkCount+1  
                subplot(linkCount+1,4, j+4*(i-1))
                hold on
                contourf(qdotMap.(jointNames{i}), torqueMap.(jointNames{i}), efficiencyMapCropped.(jointNames{i}), 10, 'ShowText', 'off', 'LabelSpacing', 800)
                plot(qdotEnvelope.(jointNames{i}), torqueEnvelope.(jointNames{i}), 'k', 'lineWidth', 2)
                s1 = scatter(qdotMotor.(EEselection)(:,i),torqueMotor.(EEselection)(:,i), markerType, 'MarkerEdgeColor', markerColorSwing, 'lineWidth', 2, 'DisplayName', labelName1);
                s2 = scatter(qdotMotor.(EEselection)(:,i),torqueMotor.(EEselection)(:,i), markerType, 'MarkerEdgeColor', markerColorStance, 'lineWidth', 2, 'DisplayName', labelName2);                
               
%                 s1 = scatter(qdotMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwing, 'MarkerEdgeColor', markerColorSwing, 'lineWidth', 2, 'DisplayName', labelName1);
%                 s2 = scatter(qdotMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStance, 'MarkerEdgeColor', markerColorStance, 'lineWidth', 2, 'DisplayName', labelName2);                
               
                if classSelection.(task).basicProperties.optimizedLegs.(EEselection)   
                    s3 = scatter(qdotMotorOpt.(EEselection)(:,i),torqueMotorOpt.(EEselection)(:,i), markerType, 'MarkerEdgeColor', markerColorSwingOpt, 'lineWidth', 2, 'DisplayName', labelName3);
                    s4 = scatter(qdotMotorOpt.(EEselection)(:,i),torqueMotorOpt.(EEselection)(:,i), markerType, 'MarkerEdgeColor', markerColorStanceOpt, 'lineWidth', 2, 'DisplayName', labelName4);                
                    
%                     s3 = scatter(qdotMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwingOpt, 'MarkerEdgeColor', markerColorSwingOpt, 'lineWidth', 2, 'DisplayName', labelName3);
%                     s4 = scatter(qdotMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStanceOpt, 'MarkerEdgeColor', markerColorStanceOpt, 'lineWidth', 2, 'DisplayName', labelName4);                
                end
                
                title(['Efficiency map for ', EEselection, ' ', (jointNames{i})])
                xlabel('qdot [rad/s]')
                ylabel('torque [Nm]')
                
                % motion averaged, design optimized 
                if classSelection.(task).basicProperties.optimizedLegs.(EEselection) 
                    legend([s1 s2 s3 s4])
                % motion averaged, design NOT optimized
                else
                    legend([s1 s2])
%                 xlim([-0.1*qdotMap.(jointNames{i})(end), 1.2*qdotMap.(jointNames{i})(end)])
%                 ylim([-0.1*torqueMap.(jointNames{i})(end), 1.2*torqueMap.(jointNames{i})(end)])
                hold off
            end
          end
end
