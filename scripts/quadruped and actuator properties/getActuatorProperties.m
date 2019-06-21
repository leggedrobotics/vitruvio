function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass] = getActuatorProperties(actuatorSelection)

if isequal(actuatorSelection,'ANYdrive')
    maxTorqueLimit = [15 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  =  [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
    actuatorMass = 1.1; % [kg]
end

if isequal(actuatorSelection,'Neo')
    maxTorqueLimit = [230 230 230 230 230];      % [Nm] HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [17 17 17 17 17];       % [rad/s] HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [2000 2000 2000 2000 2000]; % [W] These values are conservative estimates on mechanical power HAA, HFE, KFE, AFE, DFE
    actuatorMass = 4.5; % [kg]

end

if isequal(actuatorSelection,'other')
    maxTorqueLimit = [40 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
    actuatorMass = 4.5; % [kg]
end