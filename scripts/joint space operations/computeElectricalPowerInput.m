function [electricalPower, operatingPointEfficiency] = computeElectricalPowerInput(Leg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection)

    torque = Leg.(EEselection).jointTorque; % array with columns [HAA HFE KFE (AFE) (DFE)]
    qdot = Leg.(EEselection).qdot(:,1:end-1);

    % get the torque and qdot limit for the actuator at each joint
    torqueMax = [actuatorProperties.maxTorqueLimit.HAA, ...
                 actuatorProperties.maxTorqueLimit.HFE, ...
                 actuatorProperties.maxTorqueLimit.KFE];

    qdotMax =   [actuatorProperties.maxqdotLimit.HAA, ...
                 actuatorProperties.maxqdotLimit.HFE, ...
                 actuatorProperties.maxqdotLimit.KFE];

    actuator =  {actuatorSelection.HAA, ...
                 actuatorSelection.HFE, ...
                 actuatorSelection.KFE};

    lengthEfficiencyMap = [length(actuatorEfficiency.(actuator{1})), ...
                           length(actuatorEfficiency.(actuator{2})), ...
                           length(actuatorEfficiency.(actuator{3}))];

    if (linkCount > 2)
        torqueMax           = [torqueMax, actuatorProperties.maxTorqueLimit.AFE];
        qdotMax             = [qdotMax, actuatorProperties.maxqdotLimit.AFE];
        actuator            = [actuator, actuatorSelection.AFE];
        lengthEfficiencyMap = [lengthEfficiencyMap, length(actuatorEfficiency.(actuator{4}))];

        if (linkCount == 4)
            torqueMax           = [torqueMax, actuatorProperties.maxTorqueLimit.DFE];
            qdotMax             = [qdotMax, actuatorProperties.maxqdotLimit.DFE];
            actuator            = [actuator actuatorSelection.DFE];
            lengthEfficiencyMap = [lengthEfficiencyMap, length(actuatorEfficiency.(actuator{5}))];

        end
    end
    
    %% get the index in efficiency map which corresponds to the qdot, torque values at
    % each timestep
    
    qdotRatio = abs(qdot./qdotMax);
    torqueRatio = abs(torque./torqueMax);

    qdotIndex   = floor(qdotRatio .* lengthEfficiencyMap); % indexes columns in efficiency map
    torqueIndex = floor(torqueRatio .* lengthEfficiencyMap); % indexes rows in efficiency map

    qdotIndex(qdotIndex<1)     = 1; % set minimum value to 1
    torqueIndex(torqueIndex<1) = 1;

    % get efficiency at operating point for each joint and each time step
    for i = 1:linkCount+1 % joint [HAA, HFE, KFE, (AFE), (DFE)]                                              
        for j = 1:length(torqueIndex(:,i)) % time step    
            % if qdot, torque outside of bounds, set the efficiency there
            % to 0.01
            if (torqueIndex(j,i) > length(actuatorEfficiency.(actuator{i}))) || (qdotIndex(j,i) > length(actuatorEfficiency.(actuator{i})))
                operatingPointEfficiency(j,i) = 0.01;
            else
                operatingPointEfficiency(j,i) = actuatorEfficiency.(actuator{i})(torqueIndex(j,i),qdotIndex(j,i));
            end
        end
    end

    operatingPointEfficiency(operatingPointEfficiency<0.1) = 0.1; % lower bound of efficiency(?)
    mechanicalPower = qdot .* torque;
    electricalPower = mechanicalPower./operatingPointEfficiency;
    electricalPower(electricalPower<0) = 0; % set negative power terms to zero (no recuperation)
end