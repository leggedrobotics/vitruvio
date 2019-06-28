function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass] = getActuatorProperties(actuatorName)
% returns the actuator torque, speed, power limits and mass for the
% actuator present in each joint

    if isequal(actuatorName,'ANYdrive')
        maxTorqueLimit =  40;  % [Nm]   
        maxqdotLimit   =  12;  % [rad/s] 
        maxPowerLimit  = 240;  % [W]   
        actuatorMass        = 1.09;  % [kg]
    end

    if isequal(actuatorName,'Neo')
        maxTorqueLimit = 230;  % [Nm]
        maxqdotLimit   = 17;   % [rad/s]
        maxPowerLimit  = 2000; % [W] These values are conservative estimates on mechanical power
        actuatorMass        = 4.5;  % [kg]

    end

    if isequal(actuatorName,'other')
        maxTorqueLimit = 40;  % [Nm]     
        maxqdotLimit   =  12; % [rad/s] 
        maxPowerLimit  = 240; % [W] 
        actuatorMass        = 4.5; % [kg]
    end
end