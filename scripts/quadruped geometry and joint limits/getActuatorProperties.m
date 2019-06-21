function [maxTorqueLimit, maxqdotLimit, maxPowerLimit] = getActuatorProperties(actuatorSelection)

if isequal(actuatorSelection,'ANYdrive')
    maxTorqueLimit = [15 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  =  [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end

if isequal(actuatorSelection,'other')
    maxTorqueLimit = [40 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end

if isequal(actuatorSelection,'other2')
    maxTorqueLimit = [40 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end