% Units are m, °, and kg/m^3
% The mass values are used to calculate density but are not saved for
% import into NX

%% Filename and path for saved .mat file
filename = 'ANYmalBear.mat';
filepath = '/Users/michaelchadwick/Documents/git/vitruvio/scripts/cad/';

%% Robot base dimensions
robot.baseWidth.value = 0.27;
robot.baseLength.value = 0.557;
robot.baseHeight.value = 0.2592;
robot.baseMass.value = 17.1312; % Do not count HAA actuators in base mass
robot.baseDensity.value = robot.baseMass.value / (robot.baseWidth.value*robot.baseLength.value*robot.baseHeight.value);
robot.baseHeightFromGround.value = 0.5; % distance from ground to center of base in nominal stance

%% Hip attachment points (center of HAA)
robot.xNomFront.value = 0.225;
robot.xNomHind.value  = 0.225;
robot.yNomFront.value = 0.115;
robot.yNomHind.value  = 0.115;
robot.zNomFront.value = -0.071; % Positive value means HAA above center of base
robot.zNomHind.value  = -0.071;

% EE positions in nominal stance
robot.xNomEEFront.value = 0.39;
robot.xNomEEHind.value = 0.39;
robot.yNomEEFront.value = 0.155;
robot.yNomEEHind.value = 0.155;

%% Links
% Hip
robot.hipFrontLength.value = 0.1;
robot.hipHindLength.value  = 0.1;
robot.hipFrontRadius.value = 0.015;
robot.hipHindRadius.value  = 0.015;
robot.hipFrontMass.value   = 0.7702;
robot.hipHindMass.value    = 0.7702;

if robot.hipFrontLength.value > 0
    robot.hipFrontDensity.value = robot.hipFrontMass.value/(pi*robot.hipFrontRadius.value^2*robot.hipFrontLength.value);
else 
    robot.hipFrontDensity.value = 0;
end

if robot.hipHindLength.value > 0
    robot.hipHindDensity.value  = robot.hipHindMass.value/(pi*robot.hipHindRadius.value^2*robot.hipHindLength.value);
else 
    robot.hipHindDensity.value = 0;
end

% Thigh
robot.thighFrontLength.value = 0.25; %0.25;
robot.thighHindLength.value  = 0.25; %0.25;
robot.thighFrontRadius.value = 0.015;
robot.thighHindRadius.value  = 0.015;
robot.thighFrontMass.value   = 1.0296;
robot.thighHindMass.value    = 1.0296;

if robot.thighFrontLength.value > 0
    robot.thighFrontDensity.value = robot.thighFrontMass.value/(pi*robot.thighFrontRadius.value^2*robot.thighFrontLength.value);
else 
    robot.thighFrontDensity.value = 0;
end

if robot.thighHindLength.value > 0
    robot.thighHindDensity.value  = robot.thighHindMass.value/(pi*robot.thighHindRadius.value^2*robot.thighHindLength.value);
else 
    robot.thighHindDensity.value = 0;
end

% Shank
robot.shankFrontLength.value = 0.33;
robot.shankHindLength.value  = 0.33;
robot.shankFrontRadius.value = 0.015;
robot.shankHindRadius.value  = 0.015;
robot.shankFrontMass.value = 0.2072;
robot.shankHindMass.value  = 0.2072;

if robot.shankFrontLength.value > 0
    robot.shankFrontDensity.value = robot.shankFrontMass.value/(pi*robot.shankFrontRadius.value^2*robot.shankFrontLength.value);
else 
    robot.shankFrontDensity.value = 0;
end

if robot.shankHindLength.value > 0
    robot.shankHindDensity.value  = robot.shankHindMass.value/(pi*robot.shankHindRadius.value^2*robot.shankHindLength.value);
else 
    robot.shankHindDensity.value = 0;
end

%% Actuators
robot.HAAFrontMass.value = 1.09;
if robot.HAAFrontMass.value > 0 
    robot.HAAFrontRadius.value  = 0.048;
    robot.HAAFrontHeight.value  = 0.096;
    robot.HAAFrontDensity.value = robot.HAAFrontMass.value/(pi*robot.HAAFrontRadius.value^2*robot.HAAFrontHeight.value);    
else 
    robot.HAAFrontRadius.value  = 0;
    robot.HAAFrontHeight.value  = 0;
    robot.HAAFrontDensity.value = 0;    
end

robot.HAAHindMass.value = 1.09;
if robot.HAAHindMass.value > 0 
    robot.HAAHindRadius.value  = 0.048;
    robot.HAAHindHeight.value  = 0.096;
    robot.HAAHindDensity.value = robot.HAAHindMass.value/(pi*robot.HAAHindRadius.value^2*robot.HAAHindHeight.value);
else 
    robot.HAAHindRadius.value  = 0;
    robot.HAAHindHeight.value  = 0;
    robot.HAAHindDensity.value = 0;
end

robot.HFEFrontMass.value = 1.09;
if robot.HFEFrontMass.value > 0 
    robot.HFEFrontRadius.value  = 0.048;
    robot.HFEFrontHeight.value  = 0.096;
    robot.HFEFrontDensity.value = robot.HFEFrontMass.value/(pi*robot.HFEFrontRadius.value^2*robot.HFEFrontHeight.value);
else 
    robot.HFEFrontRadius.value  = 0;
    robot.HFEFrontHeight.value  = 0;
    robot.HFEFrontDensity.value = 0;
end

robot.HFEHindMass.value = 1.09;
if robot.HFEHindMass.value > 0 
    robot.HFEHindRadius.value  = 0.048;
    robot.HFEHindHeight.value  = 0.096;
    robot.HFEHindDensity.value = robot.HFEHindMass.value/(pi*robot.HFEHindRadius.value^2*robot.HFEHindHeight.value);
else 
    robot.HFEHindRadius.value  = 0;
    robot.HFEHindHeight.value  = 0;
    robot.HFEHindDensity.value = 0;
end

robot.KFEFrontMass.value = 1.09;
if robot.KFEFrontMass.value > 0 
    robot.KFEFrontRadius.value  = 0.048;
    robot.KFEFrontHeight.value  = 0.096;
    robot.KFEFrontDensity.value = robot.KFEFrontMass.value/(pi*robot.KFEFrontRadius.value^2*robot.KFEFrontHeight.value);    
else 
    robot.KFEFrontRadius.value  = 0;
    robot.KFEFrontHeight.value  = 0;
    robot.KFEFrontDensity.value = 0;    
end

robot.KFEHindMass.value = 1.09;
if robot.KFEHindMass.value > 0 
    robot.KFEHindRadius.value  = 0.048;
    robot.KFEHindHeight.value  = 0.096;
    robot.KFEHindDensity.value = robot.KFEHindMass.value/(pi*robot.KFEHindRadius.value^2*robot.KFEHindHeight.value);    
else 
    robot.KFEHindRadius.value  = 0;
    robot.KFEHindHeight.value  = 0;
    robot.HFEHindDensity.value = 0;
end

%% End effectors
robot.EEMass.value    = 0.1402;
robot.EERadius.value  = 0.026;
robot.EEDensity.value = robot.EEMass.value/((4/3)*pi*robot.EERadius.value^3);

%% Units
robot.baseWidth.unit   = 'm';
robot.baseLength.unit  = 'm';
robot.baseHeight.unit  = 'm';
robot.baseMass.unit    = 'kg';
robot.baseDensity.unit = 'kg/m^3';
robot.xNomFront.unit   = 'm';
robot.xNomHind.unit    = 'm';
robot.yNomFront.unit   = 'm';
robot.yNomHind.unit    = 'm';
robot.zNomFront.unit   = 'm';
robot.zNomHind.unit    = 'm';
robot.baseHeightFromGround.unit = 'm';

robot.xNomEEFront.unit = 'm';
robot.xNomEEHind.unit  = 'm';
robot.yNomEEFront.unit = 'm';
robot.yNomEEHind.unit  = 'm';

robot.hipFrontLength.unit  = 'm';
robot.hipHindLength.unit   = 'm';
robot.hipFrontRadius.unit  = 'm';
robot.hipHindRadius.unit   = 'm';
robot.hipFrontMass.unit    = 'kg';
robot.hipHindMass.unit     = 'kg';
robot.hipFrontDensity.unit = 'kg/m^3';
robot.hipHindDensity.unit  = 'kg/m^3';


robot.thighFrontLength.unit  = 'm';
robot.thighHindLength.unit   = 'm';
robot.thighFrontRadius.unit  = 'm';
robot.thighHindRadius.unit   = 'm';
robot.thighFrontMass.unit    = 'kg';
robot.thighHindMass.unit     = 'kg';
robot.thighFrontDensity.unit = 'kg/m^3';
robot.thighHindDensity.unit  = 'kg/m^3';

robot.shankFrontLength.unit  = 'm';
robot.shankHindLength.unit   = 'm';
robot.shankFrontRadius.unit  = 'm';
robot.shankHindRadius.unit   = 'm';
robot.shankFrontMass.unit    = 'kg';
robot.shankHindMass.unit     = 'kg';
robot.shankFrontDensity.unit = 'kg/m^3';
robot.shankHindDensity.unit  = 'kg/m^3';

robot.HAAFrontMass.unit    = 'kg';
robot.HAAFrontRadius.unit  = 'm';
robot.HAAFrontHeight.unit  = 'm';
robot.HAAHindMass.unit     = 'kg';
robot.HAAHindRadius.unit   = 'm';
robot.HAAHindHeight.unit   = 'm';
robot.HAAFrontDensity.unit = 'kg/m^3';
robot.HAAHindDensity.unit  = 'kg/m^3';

robot.HFEFrontMass.unit    = 'kg';
robot.HFEFrontRadius.unit  = 'm';
robot.HFEFrontHeight.unit  = 'm';
robot.HFEHindMass.unit     = 'kg';
robot.HFEHindRadius.unit   = 'm';
robot.HFEHindHeight.unit   = 'm';
robot.HFEFrontDensity.unit = 'kg/m^3';
robot.HFEHindDensity.unit  = 'kg/m^3';

robot.KFEFrontMass.unit    = 'kg';
robot.KFEFrontRadius.unit  = 'm';
robot.KFEFrontHeight.unit  = 'm';
robot.KFEHindMass.unit     = 'kg';
robot.KFEHindRadius.unit   = 'm';
robot.KFEHindHeight.unit   = 'm';
robot.KFEFrontDensity.unit = 'kg/m^3';
robot.KFEHindDensity.unit  = 'kg/m^3';

robot.EEMass.unit    = 'kg';
robot.EERadius.unit  ='m';
robot.EEDensity.unit ='kg/m^3';

%% Save the results to import to NX
save(filename,'robot');
% clear robot;
writeGeometry2File(filename, filepath);