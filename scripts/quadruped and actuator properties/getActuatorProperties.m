function [maxTorqueLimit, maxqdotLimit, maxPowerLimit, actuatorMass] = getActuatorProperties(actuatorSelection, linkCount)

jointNames = ['HAA'; 'HFE'; 'KFE'; 'DFE'; 'AFE'];

for i = 1:linkCount+1
    jointSelection = jointNames(i,:);
    
    if isequal(actuatorSelection.(jointSelection),'ANYdrive')
        maxTorqueLim =  40;  % [Nm]   
        maxqdotLim   =  12;  % [rad/s] 
        maxPowerLim  = 240;  % [W]   
        actMass   = 1.1;  % [kg]
    end

    if isequal(actuatorSelection.(jointSelection),'Neo')
        maxTorqueLim = 230;  % [Nm]
        maxqdotLim  = 17;   % [rad/s]
        maxPowerLim  = 2000; % [W] These values are conservative estimates on mechanical power
        actMass   = 4.5;  % [kg]

    end

    if isequal(actuatorSelection.(jointSelection),'other')
        maxTorqueLim = 40;  % [Nm]     
        maxqdotLim   =  12; % [rad/s] 
        maxPowerLim  = 240; % [W] 
        actMass   = 4.5; % [kg]
    end
    
    maxTorqueLimit.(jointSelection) = maxTorqueLim;
    maxqdotLimit.(jointSelection) = maxqdotLim;
    maxPowerLimit.(jointSelection) = maxPowerLim;
    actuatorMass.(jointSelection) = actMass;
end