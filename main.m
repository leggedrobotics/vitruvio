clear;
close all;
diary results

%% Data extraction
% if averageStepsForCyclicalMotion is true, the motion is segmented into individual steps which are averaged
% to create an average cycle. This works well when the motion is very cyclical.
% If false the individual steps are not averaged. This should be selected
% when the generated motion is irregular and highly cyclical.
dataExtraction.averageStepsForCyclicalMotion = false; 
dataExtraction.allowableDeviation = 0.1; % [m] Deviation between neighbouring points. If the deviation is larger, additional points are interpolated.

%% Toggle leg properties: leg count, link count, configuration, direct/remote joint actuation, spider/serial leg
legCount  = 4;                  % Accepts values from 1 to 4.
linkCount = 2;                  % Accepts values from 2 to 4. [thigh, shank, foot, phalanges]. Hip link connects HAA and HFE but is not included in link count.
configSelection = 'X';          % X or M

% If true, actuators are positioned in the joint which contributes to leg
% mass and inertia. If false, there is no actuator mass at joints, the 
% actuator is assumed to be in the body.
actuateJointDirectly.HAA = true; 
actuateJointDirectly.HFE = true; 
actuateJointDirectly.KFE = true;
actuateJointDirectly.AFE = false;
actuateJointDirectly.DFE = false;

%% Select actuators for each joint
% Select from: {ANYdrive, Neo, RoboDrive, Dynamixel, Other} or add a new actuator in
% getActuatorProperties
actuatorSelection.HAA = 'Neo'; 
actuatorSelection.HFE = 'Neo'; 
actuatorSelection.KFE = 'Neo';
actuatorSelection.AFE = 'Dynamixel'; 
actuatorSelection.DFE = 'Dynamixel'; 

% If joints are remotely actuated, specify the transmission method to
% compute an additional mass and inertia along all links connecting that
% joint to the body.
% Possible methods are: 'chain', 'cable', 'belt'
% The density of the chain/cable/belt is hardcoded in
% getTransmissionProperties
transmissionMethod.HAA = 'belt'; 
transmissionMethod.HFE = 'belt'; % Along hip link
transmissionMethod.KFE = 'belt'; % Along thigh link
transmissionMethod.AFE = 'belt'; % Along shank link
transmissionMethod.DFE = 'belt'; % Along foot link

% Specify hip orientation
% if true: Serial configuration. Offset from HAA to HFE parallel to the body as with ANYmal 
% if false: Spider configuration. Hip link is perpendicular to body length.
hipParalleltoBody = true;

% Simulate additional payload as point mass at CoM
payload.simulateAdditionalPayload = false;
payload.mass = 4; % kg

% Model springs in parallel with each joint.
springInParallelWithJoints = false;
% Spring constant in Nm/rad
kSpringJoint.LF = [0, 0, 1.5, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.RF = [0, 0, 1.5, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.LH = [0, 0, 10, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.RH = [0, 0, 10, 0, 0]; % HAA, HFE, KFE, AFE, DFE

%% AFE and DFE heuristics (for 3 and 4 link legs)
% The heuristic computes the final joint angle (AFE or DFE) as a 
% deformation proportional to torque. For a four link leg, the thigh and
% foot are maintained parallel.
heuristic.torqueAngle.apply = true; % Choose whether to apply the heuristic.
heuristic.torqueAngle.thetaLiftoff_des = pi/3; % Specify desired angle between final link and horizonal at liftoff. If the desired angle is impossible for the given link lengths, the closest feasible angle is obtained.
heuristic.torqueAngle.kTorsionalSpring = 50; % Spring constant for torsional spring at final joint [Nm/rad]

%% Toggle trajectory plots and initial design viz
saveFiguresToPDF             = false; % Figures are saved to results.pdf in current folder. This adds significant computation time.
robotVisualization.view       = true; % initial leg design tracking trajectory plan
robotVisualization.plotOneLeg = false;
robotVisualization.plotAllLegs = true; % If we don't average the trajectory, we can use this option to view all legs
robotVisualization.numberOfStepsVisualized = 1;    % number of steps visualized for leg motion
viewPlots.motionData         = false; % CoM position, speed. EE position and forces. Trajectory to be tracked.
viewPlots.rangeOfMotionPlots = false; % range of motion of leg for given link lengths and angle limits
viewPlots.efficiencyMap      = false; % actuator operating efficiency map
viewPlots.jointDataPlot      = false; % angle, speed, torque, power, energy data
viewPlots.metaParameterPlot  = false; % design parameters and key results plotted as pie charts

% Optimization visualization
optimizationProperties.viz.viewVisualization = true;
optimizationProperties.viz.numberOfCyclesVisualized = 1;
optimizationProperties.viz.displayBestCurrentDesign = true; % display chart of current best leg design parameters while running ga

%% Select a .mat trajectory data file to be simulated and optimized
% Select from the below options or import a new data .mat set using the
% importMotionData script

%%% Add your trajectory data file here %%%
yourTrajectoryData = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

universalTrot    = false;
universalStairs  = true;
speedyStairs     = true;
speedyGallop     = true;
massivoWalk      = true;
massivoStairs    = true;
centaurWalk      = true;
centaurStairs    = true;
miniPronk        = true;
ANYmalTrot       = false;
defaultHopperHop = false;
ANYmalSlowTrot2  = false;
ANYmalFlyingTrot = false;
ANYmalTrotVersatilityStep = false;
ANYmalSlowTrotAccurateMotion = false;

ANYmalBearTrot  = false; 
ANYmalBearTrot2 = false; % Updated to match measured data
ANYmalBearTrot3 = false; % Updated EE force splines

ANYmalBearTrotSwing3    = false; 
ANYmalBearTrotSwing5    = true; 
ANYmalBearElongatedTrot = false; 
ANYmalBearPushup        = true;
ANYmalBearSlowTrot      = true;
ANYmalBearSlowTrotIntermediateTorque = false;
ANYmalBearFlyingTrot2 = false;

vitruvianBipedPushupSquat = false;
vitruvianBipedWalk        = false;
vitruvianBipedHop         = false;
vitruvianBipedFastWalk    = false;

vertexWalk = false;
numberOfRepetitions = 0; % Number of times that leg is reoptimized. This allows for an easy check if the same optimal solution is found each time the optimization is run.

%% Toggle optimization for each leg
optimizationProperties.runOptimization = false; 
% select which legs are to be optimized
optimizeLeg.LF = true; 
optimizeLeg.RF = false; 
optimizeLeg.LH = false; 
optimizeLeg.RH = false;

%% Set optimization properties

% Set number of generations and population size
optimizationProperties.options.maxGenerations = 1;
optimizationProperties.options.populationSize = 4;

% Impose limits on maximum joint torque, speed and power
% the values are defined in getActuatorProperties. A penalty term is incurred
% for violations of these limits.
imposeJointLimits.maxTorque = true;
imposeJointLimits.maxqdot   = true;
imposeJointLimits.maxPower  = true;

% Set weights for fitness function terms. Total means summed over all
% joints in the leg.

% Penalty weights [0, inf). Zero means that term is not considered in the
% penalty function.
optimizationProperties.penaltyWeight.totalSwingTorque   = 0;
optimizationProperties.penaltyWeight.totalStanceTorque  = 0;
optimizationProperties.penaltyWeight.totalTorque        = 0;
optimizationProperties.penaltyWeight.totalTorqueHFE     = 0;
optimizationProperties.penaltyWeight.totalTorqueKFE     = 0;
optimizationProperties.penaltyWeight.swingTorqueHFE     = 0;
optimizationProperties.penaltyWeight.totalActiveTorque  = 0;
optimizationProperties.penaltyWeight.stanceActiveTorque = 0;
optimizationProperties.penaltyWeight.totalqdot          = 0;
optimizationProperties.penaltyWeight.totalPower         = 0;     % only considers power terms > 0
optimizationProperties.penaltyWeight.totalMechEnergy    = 0;
optimizationProperties.penaltyWeight.totalElecEnergy    = 0;
optimizationProperties.penaltyWeight.averageEfficiency  = 0;     % Maximizes average efficiency (even though this could increase overall energy use)
optimizationProperties.penaltyWeight.maxTorque          = 1;
optimizationProperties.penaltyWeight.maxqdot            = 0;
optimizationProperties.penaltyWeight.maxPower           = 0;     % only considers power terms > 0
optimizationProperties.penaltyWeight.antagonisticPower  = 0;     % seeks to minimize antagonistic power which improves power quality
optimizationProperties.penaltyWeight.maximumExtension   = true;  % large penalty incurred if leg extends beyond allowable amount
optimizationProperties.allowableExtension               = 0.95;   % [0 1] penalize extension above this ratio of total possible extension

% Bounds are input as multipliers of nominal input value
optimizationProperties.bounds.lowerBoundMultiplier.hipLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.hipLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.thighLength = 0.5;
optimizationProperties.bounds.upperBoundMultiplier.thighLength = 1.8;

optimizationProperties.bounds.lowerBoundMultiplier.shankLength = 0.5;
optimizationProperties.bounds.upperBoundMultiplier.shankLength = 1.8;

optimizationProperties.bounds.lowerBoundMultiplier.footLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.footLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.phalangesLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.hipOffset = 1;
optimizationProperties.bounds.upperBoundMultiplier.hipOffset = 1;

% Heuristic at AFE/DFE
optimizationProperties.bounds.lowerBoundMultiplier.kTorsionalSpring = 1;
optimizationProperties.bounds.upperBoundMultiplier.kTorsionalSpring = 1;

optimizationProperties.bounds.lowerBoundMultiplier.thetaLiftoff_des = 1;
optimizationProperties.bounds.upperBoundMultiplier.thetaLiftoff_des = 1; % with initial value pi/4 this keeps the liftoff angle on [0,pi/2]

% Transmission gear ratio from actuator output to joint
optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE = 2;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.DFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.DFE = 1;

% Spring in parallel with joint
optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.HAA = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.HAA = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.HFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.HFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.KFE = -1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.KFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.AFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.AFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.DFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.DFE = 1;

%% run the simulation
if ~optimizationProperties.runOptimization % if optimization turned off, set values to zero.
    optimizeLeg.LF = 0; optimizeLeg.RF = 0; optimizeLeg.LH = 0; optimizeLeg.RH = 0;
end
simulateSelectedTasks;
fprintf('Done.\n');

diary off