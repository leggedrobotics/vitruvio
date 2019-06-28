function [] = plotEfficiencyMapWithOperatingPoints(EEselection, actuatorEfficiency, actuatorSelection, Leg, efficiencyMapPlot, linkCount)
    
        % rows are timesteps, columns are joint [HAA, HFE, KFE, (AFE), (DFE)]
        qdot   = Leg.(EEselection).qdot;
        torque = Leg.(EEselection).jointTorque;
        
        % the name of the actuator at each joint
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
       
        %% plot
        figure('name', 'Actuator efficiency map with operating points', 'DefaultAxesFontSize', 10)
        jointNames = {'HAA'; 'HFE'; 'KFE'; 'AFE'; 'DFE'};
          for i = 1:linkCount+1  
            subplot(1, linkCount+1, i)
            hold on
            contourf(qdotMap.(jointNames{i}), torqueMap.(jointNames{i}), efficiencyMapCropped.(jointNames{i}), 10, 'ShowText', 'off', 'LabelSpacing', 800)
%           contourf(qdotMap.(jointNames{i}), torqueMap.(jointNames{i}), actuatorEfficiency.(actuator{i}), 15, 'ShowText', 'on', 'LabelSpacing', 800)
            plot(qdotEnvelope.(jointNames{i}), torqueEnvelope.(jointNames{i}), 'k', 'lineWidth', 2)
            scatter(qdot(:,i),torque(:,i), 'kx', 'lineWidth', 2)
            title(['Efficiency map for ', EEselection, ' ', (jointNames{i})])
            xlabel('qdot [rad/s]')
            ylabel('torque [Nm]')
            xlim([-qdotMap.(jointNames{i})(end), qdotMap.(jointNames{i})(end)])
            ylim([-torqueMap.(jointNames{i})(end), torqueMap.(jointNames{i})(end)])
%             xlim([-2*qdotMap.(jointNames{i})(end), 2*qdotMap.(jointNames{i})(end)])
%             ylim([-2*torqueMap.(jointNames{i})(end), 2*torqueMap.(jointNames{i})(end)])
            hold off
          end
end
