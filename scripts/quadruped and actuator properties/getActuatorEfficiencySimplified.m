function [efficiency, efficiencyMapPlot] = getActuatorEfficiencySimplified(actuatorName)
    [torqueMax, qdotMax, mechPowerMax, ~] = getActuatorProperties(actuatorName{1});

    qdotMax = qdotMax; % [kr/min]
    mechPowerMax = mechPowerMax; % [kW]
    efficiencyMax = 0.95; % efficiency map scaled such that this is the peak value
    stepSize = 0.1;
    %% compute torque envelope 
    torqueEnvelope = zeros(length(0:stepSize:qdotMax),1);
    qdotIntercept = mechPowerMax/torqueMax; % [kr/min]
    i = 1;
    % constant torque region
    for qdotEnvelope = 0:stepSize:qdotIntercept
        torqueEnvelope(i,1) = torqueMax;
        i = i+1;
    end
    % constant power region
    for qdotEnvelope = qdotIntercept+stepSize:stepSize:qdotMax
        torqueEnvelope(i,1) = mechPowerMax/qdotEnvelope; % [Nm]
        i = i+1;
    end
    qdotEnvelope = [0:stepSize:qdotMax]';
    meshDimension = length(qdotEnvelope);

    %% approximate the efficiency map with loss function
    qdot = linspace(0, qdotMax, meshDimension); % [kr/min]
    torque = (linspace(0, torqueMax, meshDimension))'; % [Nm]

    P_loss0 = 1;
    P_lossElec = (torque/torqueMax).^2.*(ones(1,meshDimension)); 
    P_lossIron = (qdot/qdotMax).^2.*(ones(meshDimension,1)); 
    P_loss3 = (torque/torqueMax).^3.*(ones(1,meshDimension)); 
    P_loss4 = (qdot/qdotMax).^3.*(ones(meshDimension,1)); 

    k0 = 1;
    k1 = 0.01;
    k2 = 0.1;
    k3 = 15;
    k4 = 15;

    % power loss terms
    P_loss  = k0*P_loss0 + k1*P_lossElec + k2*P_lossIron + k3*P_loss3 + k4*P_loss4;
    efficiency = (qdot/qdotMax).*(torque/torqueMax) ./ (((qdot/qdotMax).*(torque/torqueMax) + P_loss));

    % scale efficiency to the desired maximum efficiency value
    efficiency = efficiencyMax*efficiency/max(max(efficiency));
    [torqueOptIndex, qdotOptIndex] = find(efficiency == 0.95);
    optimalOperatingPoint = [qdot(qdotOptIndex), torque(torqueOptIndex)];

    % remove efficiency values beyond the max power limit to make plot clearer
    efficiencyMap = efficiency;
    for i = 1:length(qdot)
        for j = 1:length(torque)
            if qdot(i)*torque(j) > mechPowerMax
                efficiencyMap(j,i) = NaN;
            end
        end
    end

%     figure('name', 'Actuator efficiency map', 'DefaultAxesFontSize', 10)
%     hold on
%     contourf(qdot, torque, efficiencyMap, 15, 'ShowText', 'on', 'LabelSpacing', 800)
%     plot(qdotEnvelope, torqueEnvelope, 'k', 'lineWidth', 2)
%     plot(optimalOperatingPoint(1,1), optimalOperatingPoint(1,2), 'kx', 'LineWidth', 3)
%     title(['Efficiency map for ', actuatorName])
%     xlabel('qdot/qdotMax')
%     ylabel('torque/torqueMax')
%     hold off
%     
    % return information for plotting against operating points 
    efficiencyMapPlot.qdot = qdot;
    efficiencyMapPlot.torque = torque;
    efficiencyMapPlot.qdotEnvelope = qdotEnvelope;
    efficiencyMapPlot.torqueEnvelope = torqueEnvelope;
    efficiencyMapPlot.efficiencyMapCropped = efficiencyMap;
end