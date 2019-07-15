function [efficiency, efficiencyMapPlot] = getActuatorEfficiencySimplified(actuatorName)
    
    % get the properties of the selected actuator
    [torqueMax, qdotMax, mechPowerMax, ~, gearRatio] = getActuatorProperties(actuatorName{1});
    efficiencyMaxMotor = 0.95; % efficiency map scaled such that this is the peak value
    efficiencyMin      = 0.1; % efficiency values below efficiencyMin are set equal to efficiencyMin
    % base speed and torque for normalization of power loss terms
    qdotBase   = 0.8*qdotMax;
    torqueBase = 0.8*torqueMax;
    
    efficiencyLossGearing = 0.05;
    efficiencyMax = efficiencyMaxMotor - efficiencyLossGearing;
    stepSize = 0.1;

    %% compute torque envelope 
    torqueEnvelope = zeros(length(0:stepSize:qdotMax),1);
    qdotIntercept = mechPowerMax/torqueMax; % [kr/min]

    i = 1;
    if qdotIntercept <= qdotMax
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
    else
        % constant torque region
        for qdotEnvelope = 0:stepSize:qdotMax
            torqueEnvelope(i,1) = torqueMax;
            i = i+1;
        end
        qdotEnvelope = [0:stepSize:qdotMax]';
        meshDimension = length(qdotEnvelope);
    end

    %% approximate the efficiency map with loss function
    qdot = (linspace(0, qdotMax, meshDimension)); % [kr/min]
    torque = (linspace(0, torqueMax, meshDimension))'; % [Nm]

    % loss terms of form k_mn * torque^m * qdot^n
    P_loss00 = 1;
    P_loss10 = (torque/torqueBase)    .*(ones(1,meshDimension)); 
    P_loss20 = (torque/torqueBase).^2 .*(ones(1,meshDimension)); 
    P_loss30 = (torque/torqueBase).^3 .*(ones(1,meshDimension)); 

    P_loss01 = (qdot/qdotBase)        .* (ones(meshDimension,1));     
    P_loss11 = (torque/torqueBase)    .* (qdot/qdotBase); 
    P_loss21 = (torque/torqueBase).^2 .* (qdot/qdotBase); 
    P_loss31 = (torque/torqueBase).^3 .* (qdot/qdotBase); 
    
    P_loss02 = (qdot/qdotBase).^2     .*(ones(meshDimension,1)); 
    P_loss12 = (torque/torqueBase)    .* (qdot/qdotBase).^2; 
    P_loss22 = (torque/torqueBase).^2 .* (qdot/qdotBase).^2; 
    P_loss32 = (torque/torqueBase).^3 .* (qdot/qdotBase).^2; 
    
    P_loss03 = (qdot/qdotBase).^3     .*(ones(meshDimension,1)); 
    P_loss13 = (torque/torqueBase)    .* (qdot/qdotBase).^3; 
    P_loss23 = (torque/torqueBase).^2 .* (qdot/qdotBase).^3; 
    P_loss33 = (torque/torqueBase).^3 .* (qdot/qdotBase).^3;     

    k00 = 0.1;
    k10 = 0.1;
    k20 = 1;
    k30 = 0.1;
    
    k01 = 0.1;
    k11 = 0.01;
    k21 = 0.5;
    k31 = 0.5;
    
    k02 = 0.1;
    k12 = 0.1;
    k22 = 0.2;
    k32 = 0.2;
    
    k03 = 0.01;
    k13 = 0.1;
    k23 = 0.1;
    k33 = 0.1;
    
    P_loss = k00 * P_loss00 + ...
             k10 * P_loss10 + ...
             k20 * P_loss20 + ...
             k30 * P_loss30 + ...
             k01 * P_loss01 + ...
             k11 * P_loss11 + ...
             k21 * P_loss21 + ...
             k31 * P_loss31 + ...
             k02 * P_loss02 + ...
             k12 * P_loss12 + ...
             k22 * P_loss22 + ...
             k32 * P_loss32 + ...
             k03 * P_loss03 + ...
             k13 * P_loss13 + ...
             k23 * P_loss23 + ...
             k33 * P_loss33;    
               
    % Normalized power terms efficiency = P/(P+P_loss);     
    efficiency = (qdot/qdotBase).*(torque/torqueBase) ./ ((qdot/qdotBase).*(torque/torqueBase) + P_loss);

    % Scale efficiency to the desired maximum efficiency value
    efficiency = efficiencyMax*efficiency/max(max(efficiency));
    [torqueOptIndex, qdotOptIndex] = find(efficiency == 0.95);
    optimalOperatingPoint = [qdot(qdotOptIndex), torque(torqueOptIndex)];

    % Remove efficiency values beyond the max power limit to make plot
    % clearer. NaN region is white on contour plot.
    efficiencyMap = efficiency;
    for i = 1:length(qdot)
        for j = 1:length(torque)
            if qdot(i)*torque(j) > mechPowerMax
                efficiencyMap(j,i) = NaN;
            end
        end
    end
    
    % set minimum efficiency
    efficiencyMap(efficiencyMap<efficiencyMin) = efficiencyMin;

    % return information for plotting against operating points 
    efficiencyMapPlot.qdot = qdot;
    efficiencyMapPlot.torque = torque;
    efficiencyMapPlot.qdotEnvelope = qdotEnvelope;
    efficiencyMapPlot.torqueEnvelope = torqueEnvelope;
    efficiencyMapPlot.efficiencyMapCropped = efficiencyMap;
    efficiencyMapPlot.qdotIntercept = qdotIntercept;
end