function [maxTorqueLimit, maxqdotLimit, maxPowerLimit] = getActuatorLimits(classSelection)

if isequal(classSelection,'ANYmal')
    maxTorqueLimit = [15 40 20 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end

if isequal(classSelection,'universal')
    maxTorqueLimit = [40 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end

if isequal(classSelection,'speedy')
    maxTorqueLimit = [40 40 40 40 40];      % [Nm]     HAA, HFE, KFE, AFE, DFE
    maxqdotLimit  = [12 12 12 12 12];       % [rad/s]  HAA, HFE, KFE, AFE, DFE
    maxPowerLimit  = [240 240 240 240 240]; % [W]      HAA, HFE, KFE, AFE, DFE
end