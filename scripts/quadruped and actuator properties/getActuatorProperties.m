function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass, gearRatio] = getActuatorProperties(actuatorName)
% returns the actuator torque, speed, power limits and mass for the
% actuator present in each joint

%%% Add your actuator properties here %%%
    if isequal(actuatorName,'yourActuator')
        maxTorqueLimit = 10;  % [Nm]   
        maxqdotLimit   = 10;  % [rad/s] 
        maxPowerLimit  = 100;  % [W]   
        actuatorMass   = 1;  % [kg]
        gearRatio      = 10;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    if isequal(actuatorName,'ANYdrive')
        maxTorqueLimit = 40;  % [Nm]   
        maxqdotLimit   = 12;  % [rad/s] 
        maxPowerLimit  = 240;  % [W]   
        actuatorMass   = 1.09;  % [kg]
        gearRatio      = 50;
    end

    if isequal(actuatorName,'Neo')
        maxTorqueLimit = 230;  % [Nm]
        maxqdotLimit   = 17;   % [rad/s]
        maxPowerLimit  = 4000; % [W] 
        actuatorMass   = 4.5;  % [kg]
        gearRatio      = 6.6;
    end
    
    if isequal(actuatorName,'RoboDrive') % RoboDrive 115x25
        maxTorqueLimit = 70;  % [Nm]     
        maxqdotLimit   = 40; % [rad/s] 
        maxPowerLimit  = 4200; % [W] 
        actuatorMass   = 3; % [kg]
        gearRatio      = 4.5;   
    end
    
    if isequal(actuatorName,'other')
        maxTorqueLimit = 40;  % [Nm]     
        maxqdotLimit   = 12; % [rad/s] 
        maxPowerLimit  = 240; % [W] 
        actuatorMass   = 4.5; % [kg]
        gearRatio      = 50;        
    end
end