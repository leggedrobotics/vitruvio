function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass, gearRatio, efficiencyMinMotor, efficiencyMaxMotor] = getActuatorProperties(actuatorName)
% returns the actuator torque, speed, power limits and mass for the
% actuator present in each joint

%%% Add your actuator properties here %%%
    if isequal(actuatorName,'yourActuator')
        maxTorqueLimit     = 10;  % [Nm]   
        maxqdotLimit       = 10;  % [rad/s] 
        maxPowerLimit      = 100; % [W]   
        actuatorMass       = 1;   % [kg]
        gearRatio          = 10;
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.9; % motor efficiency before gearing losses
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if isequal(actuatorName,'Dynamixel64R') % Dynamixel MX-64R
        maxTorqueLimit = 7.3;  % [Nm]   
        maxqdotLimit   = 6.6;  % [rad/s] 
        maxPowerLimit  = 20;  % [W] Not sure about this value 
        actuatorMass   = 0.126;  % [kg]
        gearRatio      = 200;
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.95; % motor efficiency before gearing losses        
    end
    
    if isequal(actuatorName,'DynamixelXM540') % Dynamixel XM-540
        maxTorqueLimit = 10.6;  % [Nm]   
        maxqdotLimit   = 3.14;  % [rad/s] 
        maxPowerLimit  = 20;  % [W] Not sure about this value 
        actuatorMass   = 0.126;  % [kg]
        gearRatio      = 200; % check this
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.95; % motor efficiency before gearing losses        
    end
    
    if isequal(actuatorName,'ANYdrive')
        maxTorqueLimit = 40;  % [Nm]   
        maxqdotLimit   = 12;  % [rad/s] 
        maxPowerLimit  = 240;  % [W]   
        actuatorMass   = 1.09;  % [kg]
        gearRatio      = 50;
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.97; % motor efficiency before gearing losses        
    end

    if isequal(actuatorName,'Neo')
        maxTorqueLimit = 230;  % [Nm]
        maxqdotLimit   = 17;   % [rad/s]
        maxPowerLimit  = 4000; % [W] 
        actuatorMass   = 4.5;  % [kg]
        gearRatio      = 6.6;
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.95; % motor efficiency before gearing losses        
    end
    
    if isequal(actuatorName,'RoboDrive') % RoboDrive 115x25
        maxTorqueLimit = 70;  % [Nm]     
        maxqdotLimit   = 40; % [rad/s] 
        maxPowerLimit  = 4200; % [W] 
        actuatorMass   = 3; % [kg]
        gearRatio      = 4.5;  
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 1; % motor efficiency before gearing losses        
    end
    
    if isequal(actuatorName,'Other')
        maxTorqueLimit = 40;  % [Nm]     
        maxqdotLimit   = 12; % [rad/s] 
        maxPowerLimit  = 240; % [W] 
        actuatorMass   = 4.5; % [kg]
        gearRatio      = 50;    
        efficiencyMinMotor = 0.1;  
        efficiencyMaxMotor = 0.95; % motor efficiency before gearing losses        
    end
end