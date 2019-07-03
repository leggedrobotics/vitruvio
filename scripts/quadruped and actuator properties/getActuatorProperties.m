function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass, gearRatio] = getActuatorProperties(actuatorName)
% returns the actuator torque, speed, power limits and mass for the
% actuator present in each joint

    if isequal(actuatorName,'ANYdrive')
        maxTorqueLimit =  40;  % [Nm]   
        maxqdotLimit   =  12;  % [rad/s] 
        maxPowerLimit  = 240;  % [W]   
        actuatorMass   = 1.09;  % [kg]
        gearRatio      = 50;
    end

    if isequal(actuatorName,'Neo')
        maxTorqueLimit = 230;  % [Nm]
        maxqdotLimit   = 17;   % [rad/s]
        maxPowerLimit  = 2000; % [W] These values are conservative estimates on mechanical power
        actuatorMass   = 4.5;  % [kg]
        gearRatio      = 9.55;
    end
    
    if isequal(actuatorName,'Capler')
        maxTorqueLimit = 70;  % [Nm]     
        maxqdotLimit   = 40; % [rad/s] 
        maxPowerLimit  = 4200; % [W] 
        actuatorMass   = 3; % [kg]
        gearRatio      = 50;    % Get this number from Hendrik     
    end
    
    if isequal(actuatorName,'other')
        maxTorqueLimit = 40;  % [Nm]     
        maxqdotLimit   =  12; % [rad/s] 
        maxPowerLimit  = 240; % [W] 
        actuatorMass   = 4.5; % [kg]
        gearRatio      = 50;        
    end
end