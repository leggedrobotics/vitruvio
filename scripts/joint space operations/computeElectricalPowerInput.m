function [electricalPower, operatingPointEfficiency] = computeElectricalPowerInput(Leg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection)

    % Actuator torque and speed values after applying gear and spring at joint
    torque = Leg.(EEselection).actuatorTorque; 
    qdot = Leg.(EEselection).actuatorqdot;

    % get the actuator limits for the actuator at each joint
    torqueMax = [actuatorProperties.maxTorqueLimit.HAA, ...
                 actuatorProperties.maxTorqueLimit.HFE, ...
                 actuatorProperties.maxTorqueLimit.KFE];

    qdotMax =   [actuatorProperties.maxqdotLimit.HAA, ...
                 actuatorProperties.maxqdotLimit.HFE, ...
                 actuatorProperties.maxqdotLimit.KFE];
             
    powerMax =  [actuatorProperties.maxPowerLimit.HAA, ...
                 actuatorProperties.maxPowerLimit.HFE, ...
                 actuatorProperties.maxPowerLimit.KFE];            

    efficiencyMin.(actuatorSelection.HAA) = actuatorProperties.efficiencyMinMotor.HAA;
    efficiencyMin.(actuatorSelection.HFE) = actuatorProperties.efficiencyMinMotor.HFE;
    efficiencyMin.(actuatorSelection.KFE) = actuatorProperties.efficiencyMinMotor.KFE;
    
    efficiencyMax.(actuatorSelection.HAA) = actuatorProperties.efficiencyMaxMotor.HAA;
    efficiencyMax.(actuatorSelection.HFE) = actuatorProperties.efficiencyMaxMotor.HFE;
    efficiencyMax.(actuatorSelection.KFE) = actuatorProperties.efficiencyMaxMotor.KFE;    
                   
    actuator =  {actuatorSelection.HAA, ...
                 actuatorSelection.HFE, ...
                 actuatorSelection.KFE};

    lengthEfficiencyMap = [length(actuatorEfficiency.(actuator{1})), ...
                           length(actuatorEfficiency.(actuator{2})), ...
                           length(actuatorEfficiency.(actuator{3}))];

    if (linkCount > 2)
        torqueMax           = [torqueMax, actuatorProperties.maxTorqueLimit.AFE];
        qdotMax             = [qdotMax, actuatorProperties.maxqdotLimit.AFE];
        powerMax            = [powerMax, actuatorProperties.maxPowerLimit.AFE];
        efficiencyMin.(actuatorSelection.AFE)   = actuatorProperties.efficiencyMinMotor.AFE;    
        efficiencyMax.(actuatorSelection.AFE)   = actuatorProperties.efficiencyMaxMotor.AFE;    
        actuator            = [actuator, actuatorSelection.AFE];
        lengthEfficiencyMap = [lengthEfficiencyMap, length(actuatorEfficiency.(actuator{4}))];

        if (linkCount == 4)
            torqueMax           = [torqueMax, actuatorProperties.maxTorqueLimit.DFE];
            qdotMax             = [qdotMax, actuatorProperties.maxqdotLimit.DFE];
            powerMax            = [powerMax, actuatorProperties.maxPowerLimit.DFE];
            efficiencyMin.(actuatorSelection.DFE)   = actuatorProperties.efficiencyMinMotor.DFE;                
            efficiencyMax.(actuatorSelection.DFE)   = actuatorProperties.efficiencyMaxMotor.DFE;    
            actuator            = [actuator actuatorSelection.DFE];
            lengthEfficiencyMap = [lengthEfficiencyMap, length(actuatorEfficiency.(actuator{5}))];
        end
    end
    
    %% Determine the efficiency at each operating point
    % If points are in quadrant II, IV, or outside of actuator limits, set
    % their efficiency to efficiencyMin.
    
    % Get the index in efficiency map which corresponds to the qdot, torque values
    qdotRatio   = qdot./qdotMax; % Each column of qdot divided by corresponding qdotMax value for that actuator
    torqueRatio = torque./torqueMax;
    powerRatio  = qdot.*torque ./ powerMax;
    
    % qdotIndex and torqueIndex have length qdot and torque.
    qdotIndex   = floor(qdotRatio .* lengthEfficiencyMap); % indexes columns in efficiency map
    torqueIndex = floor(torqueRatio .* lengthEfficiencyMap); % indexes rows in efficiency map
    qdotIndex(qdotIndex<1)     = 1; % Set minimum index values to 1
    torqueIndex(torqueIndex<1) = 1;
    
    % Get efficiency at operating point for each joint and each time step
    for i = 1:linkCount+1 % joint [HAA, HFE, KFE, (AFE), (DFE)]                                              
        for j = 1:length(torqueIndex(:,i)) % time step  
            if torqueRatio(j,i)*qdotRatio(j,i) < 0 % Set efficiency for Quadruant II and IV to the minimum value.
                operatingPointEfficiency(j,i) = efficiencyMin.(actuator{i});
            else
                % Now looking at Quadrant I and III.
                % Set efficiency in regions outside of actuator limits to
                % mininum efficiency.
                if torqueRatio(j,i) > 1 || qdotRatio(j,i) > 1 || powerRatio(j,i) > 1
                    operatingPointEfficiency(j,i) = efficiencyMin.(actuator{i});
                else 
                    operatingPointEfficiency(j,i) = actuatorEfficiency.(actuator{i})(torqueIndex(j,i),qdotIndex(j,i));
                end
            end
        end
    end
    
    mechanicalPower = qdot .* torque; % of active component of torque
    electricalPower = mechanicalPower./operatingPointEfficiency;
    electricalPower(electricalPower<0) = 0; % set negative power terms to zero (no recuperation)
end