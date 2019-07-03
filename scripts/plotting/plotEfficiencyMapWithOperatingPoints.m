function [] = plotEfficiencyMapWithOperatingPoints(viewOptimizedData, EEselectionOpt, EEnames, actuatorSelection, Leg, efficiencyMapPlot, actuatorProperties, linkCount, meanTouchdownIndex, dataExtraction)

    % if viewOptimizedLegPlot is true, the same operations are also performed
    % on the optimized design data to plot this data against that of the
    % initial design.
    if viewOptimizedData
        qdotOpt.(EEselectionOpt)   = Leg.(EEselectionOpt).qdotOpt;
        torqueOpt.(EEselectionOpt) = Leg.(EEselectionOpt).jointTorqueOpt;

        qdotMotorOpt.(EEselectionOpt)(:,1) = qdotOpt.(EEselectionOpt)(:,1)*actuatorProperties.gearRatio.HAA;        
        qdotMotorOpt.(EEselectionOpt)(:,2) = qdotOpt.(EEselectionOpt)(:,2)*actuatorProperties.gearRatio.HFE;
        qdotMotorOpt.(EEselectionOpt)(:,3) = qdotOpt.(EEselectionOpt)(:,3)*actuatorProperties.gearRatio.KFE;

        torqueMotorOpt.(EEselectionOpt)(:,1) = torqueOpt.(EEselectionOpt)(:,1)/actuatorProperties.gearRatio.HAA;
        torqueMotorOpt.(EEselectionOpt)(:,2) = torqueOpt.(EEselectionOpt)(:,2)/actuatorProperties.gearRatio.HFE;
        torqueMotorOpt.(EEselectionOpt)(:,3) = torqueOpt.(EEselectionOpt)(:,3)/actuatorProperties.gearRatio.KFE;

        if linkCount > 2
            qdotMotorOpt.(EEselectionOpt)(:,4)   = qdotOpt.(EEselectionOpt)(:,4)*actuatorProperties.gearRatio.AFE;
            torqueMotorOpt.(EEselectionOpt)(:,4) = torqueOpt.(EEselectionOpt)(:,4)/actuatorProperties.gearRatio.AFE; 
        end
        if linkCount == 4
                qdotMotorOpt.(EEselectionOpt)(:,5)   = qdotOpt.(EEselectionOpt)(:,5)*actuatorProperties.gearRatio.DFE;
                torqueMotorOpt.(EEselectionOpt)(:,5) = torqueOpt.(EEselectionOpt)(:,5)/actuatorProperties.gearRatio.DFE;
        end
        %% Reflect points from quadrant III to quadrant I
        for j = 1:length(qdotMotorOpt.(EEselectionOpt)(1,:)) % joint
            for i = 1:length(qdotMotorOpt.(EEselectionOpt)(:,1)) % timestep
                if (qdotMotorOpt.(EEselectionOpt)(i,j)<0) && (torqueMotorOpt.(EEselectionOpt)(i,j)<0)
                    qdotMotorOpt.(EEselectionOpt)(i,j) = abs(qdotMotorOpt.(EEselectionOpt)(i,j));
                    torqueMotorOpt.(EEselectionOpt)(i,j) = abs(torqueMotorOpt.(EEselectionOpt)(i,j));
                end
                if viewOptimizedData
                    if (qdotMotorOpt.(EEselectionOpt)(i,j)<0) && (torqueMotorOpt.(EEselectionOpt)(i,j)<0)
                        qdotMotorOpt.(EEselectionOpt)(i,j) = abs(qdotMotorOpt.(EEselectionOpt)(i,j));
                        torqueMotorOpt.(EEselectionOpt)(i,j) = abs(torqueMotorOpt.(EEselectionOpt)(i,j));
                    end
                end
            end
        end
    end
    
    for i = 1:4
        EEselection = EEnames(i,:);
        % rows are timesteps, columns are joint [HAA, HFE, KFE, (AFE), (DFE)]
        qdot.(EEselection)   = Leg.(EEselection).qdot;
        torque.(EEselection) = Leg.(EEselection).jointTorque;
              
        %% convert the qdot and torque values from actuator to motor level
        qdotMotor.(EEselection)(:,1) = qdot.(EEselection)(:,1)*actuatorProperties.gearRatio.HAA;        
        qdotMotor.(EEselection)(:,2) = qdot.(EEselection)(:,2)*actuatorProperties.gearRatio.HFE;
        qdotMotor.(EEselection)(:,3) = qdot.(EEselection)(:,3)*actuatorProperties.gearRatio.KFE;
        
        torqueMotor.(EEselection)(:,1) = torque.(EEselection)(:,1)/actuatorProperties.gearRatio.HAA;
        torqueMotor.(EEselection)(:,2) = torque.(EEselection)(:,2)/actuatorProperties.gearRatio.HFE;
        torqueMotor.(EEselection)(:,3) = torque.(EEselection)(:,3)/actuatorProperties.gearRatio.KFE;

        if linkCount > 2
            qdotMotor.(EEselection)(:,4)   = qdot.(EEselection)(:,4)*actuatorProperties.gearRatio.AFE;
            torqueMotor.(EEselection)(:,4) = torque.(EEselection)(:,4)/actuatorProperties.gearRatio.AFE;            
        end
        if linkCount == 4
            qdotMotor.(EEselection)(:,5)   = qdot.(EEselection)(:,5)*actuatorProperties.gearRatio.DFE;
            torqueMotor.(EEselection)(:,5) = torque.(EEselection)(:,5)/actuatorProperties.gearRatio.DFE; 
        end

        %% Reflect points from quadrant III to quadrant I
        for j = 1:length(qdotMotor.(EEselection)(1,:)) % joint
            for i = 1:length(qdotMotor.(EEselection)(:,1)) % timestep
                if (qdotMotor.(EEselection)(i,j)<0) && (torqueMotor.(EEselection)(i,j)<0)
                    qdotMotor.(EEselection)(i,j) = abs(qdotMotor.(EEselection)(i,j));
                    torqueMotor.(EEselection)(i,j) = abs(torqueMotor.(EEselection)(i,j));
                end
            end
        end
        
        %% the name of the actuator at each joint
        actuator =  {actuatorSelection.HAA, ...
                     actuatorSelection.HFE, ...
                     actuatorSelection.KFE};         
        if linkCount == 3
            actuator =  [actuator, actuatorSelection.AFE];
        elseif linkCount == 4
             actuator =  [actuator, actuatorSelection.AFE, actuatorSelection.DFE];
        end
        
        %% save the values for plotting the envelope and efficiency map
        qdotEnvelope.HAA         = efficiencyMapPlot.(actuator{1}).qdotEnvelope;
        torqueEnvelope.HAA       = efficiencyMapPlot.(actuator{1}).torqueEnvelope;  
        qdotMap.HAA              = efficiencyMapPlot.(actuator{1}).qdot;
        torqueMap.HAA            = efficiencyMapPlot.(actuator{1}).torque;  
        efficiencyMapCropped.HAA = efficiencyMapPlot.(actuator{1}).efficiencyMapCropped;  
        
        qdotEnvelope.HFE         = efficiencyMapPlot.(actuator{2}).qdotEnvelope;
        torqueEnvelope.HFE       = efficiencyMapPlot.(actuator{2}).torqueEnvelope; 
        qdotMap.HFE              = efficiencyMapPlot.(actuator{2}).qdot;
        torqueMap.HFE            = efficiencyMapPlot.(actuator{2}).torque; 
        efficiencyMapCropped.HFE = efficiencyMapPlot.(actuator{2}).efficiencyMapCropped;  
        
        qdotEnvelope.KFE         = efficiencyMapPlot.(actuator{3}).qdotEnvelope;
        torqueEnvelope.KFE       = efficiencyMapPlot.(actuator{3}).torqueEnvelope;   
        qdotMap.KFE              = efficiencyMapPlot.(actuator{3}).qdot;
        torqueMap.KFE            = efficiencyMapPlot.(actuator{3}).torque;     
        efficiencyMapCropped.KFE = efficiencyMapPlot.(actuator{3}).efficiencyMapCropped;  
        
        if linkCount > 2
            qdotEnvelope.AFE         = efficiencyMapPlot.(actuator{4}).qdotEnvelope;
            torqueEnvelope.AFE       = efficiencyMapPlot.(actuator{4}).torqueEnvelope;   
            qdotMap.AFE              = efficiencyMapPlot.(actuator{4}).qdot;
            torqueMap.AFE            = efficiencyMapPlot.(actuator{4}).torque;     
            efficiencyMapCropped.AFE = efficiencyMapPlot.(actuator{4}).efficiencyMapCropped;  
        end
        
        if linkCount == 4
            qdotEnvelope.DFE         = efficiencyMapPlot.(actuator{5}).qdotEnvelope;
            torqueEnvelope.DFE       = efficiencyMapPlot.(actuator{5}).torqueEnvelope;   
            qdotMap.DFE              = efficiencyMapPlot.(actuator{5}).qdot;
            torqueMap.DFE            = efficiencyMapPlot.(actuator{5}).torque;         
            efficiencyMapCropped.DFE = efficiencyMapPlot.(actuator{5}).efficiencyMapCropped;  
        end
    end
    
    if dataExtraction.averageStepsForCyclicalMotion  % if true, the points are averaged and we can differentiate swing and stance
        markerTypeSwing     = 'x';
        markerColorSwing    = 'r';
        labelName1          = 'initial swing phase';
        
        markerTypeStance    = 'x';
        markerColorStance   = '[0.5430 0 0]'; % dark red
        labelName2          = 'initial stance phase';
        
        markerTypeSwingOpt  = 'x';
        markerColorSwingOpt = 'b';
        labelName3          = 'optimized swing phase';
        
        markerTypeStanceOpt  = 'x';
        markerColorStanceOpt = '[0 0 0.5430]'; % dark blue 
        labelName4           = 'optimized stance phase';        
    elseif ~dataExtraction.averageStepsForCyclicalMotion && viewOptimizedData 
        markerTypeSwing    = 'x';
        markerColorSwing   = 'r';
        labelName1         = 'initial leg design';
       
        markerTypeStance    = 'x';
        markerColorStance   = 'r';
        labelName2         = 'initial leg design';
        
        markerTypeSwingOpt    = 'x';
        markerColorSwingOpt   = 'b';
        labelName3         = 'optimized leg design';
        
        markerTypeStanceOpt    = 'x';
        markerColorStanceOpt  = 'b';
        labelName4         = 'optimized leg design';
    else % leg not optimized but the points are averaged so we have distinct swing and stance phases
        markerTypeSwing    = 'x';
        markerColorSwing   = 'r';
        labelName1         = 'swing phase';
       
        markerTypeStance    = 'x';
        markerColorStance   = '[0.5430 0 0]';
        labelName2         = 'stance phase';
    end
     
        %% plot
        figure('name', 'Actuator efficiency map with operating points', 'DefaultAxesFontSize', 10)
        jointNames = {'HAA'; 'HFE'; 'KFE'; 'AFE'; 'DFE'};
          for j = 1:4
            EEselection = EEnames(j,:);
            for i = 1:linkCount+1  
                subplot(linkCount+1,4, j+4*(i-1))
                hold on
                contourf(qdotMap.(jointNames{i}), torqueMap.(jointNames{i}), efficiencyMapCropped.(jointNames{i}), 10, 'ShowText', 'off', 'LabelSpacing', 800)
%                 plot(qdotEnvelope.(jointNames{i}), torqueEnvelope.(jointNames{i}), 'k', 'lineWidth', 2)
               
                s1 = scatter(qdotMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotor.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwing, 'MarkerEdgeColor', markerColorSwing, 'lineWidth', 2, 'DisplayName', labelName1);
                s2 = scatter(qdotMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotor.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStance, 'MarkerEdgeColor', markerColorStance, 'lineWidth', 2, 'DisplayName', labelName2);                
               
                if (viewOptimizedData && isequal(EEselection, EEselectionOpt))
                        s3 = scatter(qdotMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i),torqueMotorOpt.(EEselection)(1:meanTouchdownIndex.(EEselection)-1,i), markerTypeSwingOpt, 'MarkerEdgeColor', markerColorSwingOpt, 'lineWidth', 2, 'DisplayName', labelName3);
                        s4 = scatter(qdotMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i),torqueMotorOpt.(EEselection)(meanTouchdownIndex.(EEselection):end,i), markerTypeStanceOpt, 'MarkerEdgeColor', markerColorStanceOpt, 'lineWidth', 2, 'DisplayName', labelName4);                
                end
                
                title(['Efficiency map for ', EEselection, ' ', (jointNames{i})])
                xlabel('qdot [rad/s]')
                ylabel('torque [Nm]')
                % motion averaged, design optimized 
                if (dataExtraction.averageStepsForCyclicalMotion && viewOptimizedData && isequal(EEselection, EEselectionOpt))
                    legend([s1 s2 s3 s4])
                % motion NOT averaged, design optimized 
                elseif (viewOptimizedData && isequal(EEselection, EEselectionOpt))
                    legend([s1 s3])
                % motion averaged, design NOT optimized
                else
                    legend([s1 s2])
                xlim([-0.1*qdotMap.(jointNames{i})(end), 1.2*qdotMap.(jointNames{i})(end)])
                ylim([-0.1*torqueMap.(jointNames{i})(end), 1.2*torqueMap.(jointNames{i})(end)])
                hold off
            end
          end
end
