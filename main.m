clear all;
close all;
clc; 

recordCommandWindow = false; % If true command window is recorded and saved in txt file

%% DATA EXTRACTION
% if averageStepsForCyclicalMotion is true, the motion is segmented into individual steps which are averaged
% to create a single step cycle - this works well when the motion is highly cyclical.
% If false the individual steps are not averaged - this is preferred
% when the generated motion is irregular.
dataExtraction.averageStepsForCyclicalMotion = false; 
dataExtraction.allowableDeviation = 0.05; % [m] Deviation between neighbouring points. If the deviation is larger, additional points are interpolated.

%% LEG PROPERTIES
legCount  = 4;                  % Accepts values from 1 to 4.
linkCount = 2;                  % Accepts values from 2 to 4. [thigh, shank, foot, phalanges]. Hip link connects HAA and HFE but is not included in link count.
configSelection = 'X';          % X or M. By default, the HFE is outwards from HAA but for M configuration this may not be desired. To avoid this set the hip length to negative in getRobotProperties.m.

% If true, actuators are positioned in the joint which contributes to leg
% mass and inertia. If false, there is no actuator mass at joints, the 
% actuator is assumed to be in the body.
actuateJointDirectly.HAA = true; 
actuateJointDirectly.HFE = true; 
actuateJointDirectly.KFE = true;
actuateJointDirectly.AFE = true;
actuateJointDirectly.DFE = true;

%% ACTUATOR SELECTION
% Select from: {ANYdrive, Neo, RoboDrive, Dynamixel64R, DynamixelXM540, Other} or add a new actuator in
% getActuatorProperties
actuatorSelection.HAA = 'ANYdrive'; 
actuatorSelection.HFE = 'ANYdrive'; 
actuatorSelection.KFE = 'ANYdrive';
actuatorSelection.AFE = 'ANYdrive'; 
actuatorSelection.DFE = 'ANYdrive'; 

%% TRANSMISSION
% If joints are remotely actuated, specify the transmission method to
% compute an additional mass and inertia along all links connecting that
% joint to the body.
% Possible methods are: 'chain', 'cable', 'belt'
% The density of the chain/cable/belt is hardcoded in
% getTransmissionProperties and should be updated according to available
% components.
transmissionMethod.HAA = 'belt'; 
transmissionMethod.HFE = 'belt'; % Along hip link
transmissionMethod.KFE = 'belt'; % Along thigh link
transmissionMethod.AFE = 'belt'; % Along shank link
transmissionMethod.DFE = 'belt'; % Along foot link

%% HIP ORIENTATION
% if true: Mammal configuration. Offset from HAA to HFE parallel to the body as with ANYmal 
% if false: Spider configuration. Hip link is perpendicular to body length.
hipParalleltoBody = true;

%% SIMULATE ADDITIONAL PAYLOAD
% Simulate additional payload as point mass at CoM. Approximates a payload
% without rerunning the trajectory optimization.
payload.simulateAdditionalPayload = false;
payload.mass = 10; % kg

%% PARALLEL SPRINGS
% Model springs in parallel with selected joints.
springInParallelWithJoints = false;
% Apply if springInParallelWithJoints = true, select the spring constant in
% Nm/rad for each joint
kSpringJoint.LF = [0, 0, 100, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.RF = [0, 0, 100, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.LH = [0, 0, 100, 0, 0]; % HAA, HFE, KFE, AFE, DFE
kSpringJoint.RH = [0, 0, 100, 0, 0]; % HAA, HFE, KFE, AFE, DFE
% Specify spring set point q0
q0SpringJoint.useAverageJointPosition = true; % If true, the spring set point is set to the joint's average position. If false, set point specified below.
q0SpringJoint.LF = [0, 0, 0, 0, 0]; % HAA, HFE, KFE, AFE, DFE
q0SpringJoint.RF = [0, 0, 0, 0, 0]; % HAA, HFE, KFE, AFE, DFE
q0SpringJoint.LH = [0, 0, 0, 0, 0]; % HAA, HFE, KFE, AFE, DFE
q0SpringJoint.RH = [0, 0, 0, 0, 0]; % HAA, HFE, KFE, AFE, DFE

%% MOTOR EFFICIENCY MODEL (Under development)
% Compute efficiency map for actuators. The actuator efficiency maps
% require tuning for each actuator to ensure they are accurate.
computeEfficiencyMaps = false;

%% AFE/DFE HEURISTICS (for 3 and 4 link legs)
% The heuristic computes the final joint angle (AFE or DFE) as a 
% deformation proportional to torque to simulate a spring at that 
% location without an actuator. For a four link leg, the thigh and
% foot are maintained parallel.
heuristic.torqueAngle.apply = false; % Choose whether to apply the heuristic.
heuristic.torqueAngle.thetaLiftoff_des = pi/4; % Specify desired angle between final link and horizonal at liftoff. If the desired angle is impossible for the given link lengths, the closest feasible angle is obtained.
heuristic.torqueAngle.kTorsionalSpring = 100; % Spring constant for torsional spring at final joint [Nm/rad]

%% VISUALIZATION OPTIONS
saveFiguresToPDF                 = false;  % Figures are saved to results.pdf in current folder. This adds significant computation time.
robotVisualization.view          = false;  % Visualization of nominal robot
    robotVisualization.oneLeg    = false;  % View a single leg tracking the trajectory.
    robotVisualization.allLegs   = true;   % View motion with all legs (incompatible with averageStepsForCyclicalMotion = true)
    robotVisualization.torso     = false;  % Also displays a torso at the front of the robot, dimensions defined in visualizeRobot.m
    robotVisualization.createGif = false;  % Creates a gif of the motion
    
viewPlots.motionData           = true;  % CoM position, speed. EE position and forces. Trajectory to be tracked.
viewPlots.efficiencyMap        = false; % (In development) Actuator operating efficiency map. Requires computeEfficiencyMaps = true
viewPlots.jointDataPlot        = true;  % By default only joint level speed, torque, power, and energy are plotted. Actuator/motor/spring level plots can be turned on in plotJointDataForAllLegs.m.
   % Along with the joint level plot, actuator level and motor level plots
   % may also be displayed. If springs are present, a plot of the active
   % and passive torque will also be displayed.
    viewPlots.displayActuatorLevelPlots = false;
    viewPlots.displayMotorLevelPlots = false;
    
optimizationProperties.viz.displayBestCurrentDesign = true; % Display chart of current best leg design parameters while running ga

%% DATA SELECTION
% Select a .mat trajectory data file to be simulated and optimized
% Select from the below options or import a new data .mat set using the
% importMotionData.m script

%%% Add your trajectory data file here with name class_task %%%
dataSelection.yourTrajectoryData = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataSelection.ANYmalBear_fastTrot = true; 
dataSelection.mini_pronk          = false;
dataSelection.hopper_hop          = false;

optimizationCount = 1; % Set to 1 to run the optimization only once. For values >1 the leg is reoptimized with the same settings. This allows for an easy check if the same optimal solution is found each time the optimization is run.

%% RUN OPTIMIZATION
optimizationProperties.runOptimization = false; % If true, selected legs will be optimized.
% Select which legs are to be optimized
optimizeLeg.LF = true; 
optimizeLeg.RF = false; 
optimizeLeg.LH = false; 
optimizeLeg.RH = false;

%% OPTIMIZATION OPTIONS
% Set number of generations and population size for the optimization.
optimizationProperties.options.maxGenerations = 10;
optimizationProperties.options.populationSize = 10;

% Impose limits on maximum joint torque, speed and power.
% The limiting values are defined for each actuator in getActuatorProperties. A penalty term is incurred
% for violations of these limits.
imposeJointLimits.maxTorque = false;
imposeJointLimits.maxqdot   = false;
imposeJointLimits.maxPower  = false;
imposeJointLimits.limitingValue = 0.9; % Specified as a ratio of the actuator limit. Penalize when actuators loaded beyond this ratio.

% Set weights for cost function terms. Total means summed over all joints in the leg.
% Penalty weights [0, inf). Zero means that term is not considered in the
% penalty function. Unless otherwise specified, the speed/torque/power/energy terms are penalized at actuator level.
% When the tranmission ratio is 1, actuator and joint level are equivalent.
optimizationProperties.penaltyWeight.totalSwingTorque   = 0;   % Terms which penalize swing/stance are only used when averageStepsForCyclicalMotion = true
optimizationProperties.penaltyWeight.totalStanceTorque  = 0;
optimizationProperties.penaltyWeight.totalTorque        = 1;
optimizationProperties.penaltyWeight.totalTorqueHFE     = 0;
optimizationProperties.penaltyWeight.totalTorqueKFE     = 0;
optimizationProperties.penaltyWeight.swingTorqueHFE     = 0;
optimizationProperties.penaltyWeight.totalActiveTorque  = 0;
optimizationProperties.penaltyWeight.stanceActiveTorque = 0;
optimizationProperties.penaltyWeight.totalqdot          = 0;
optimizationProperties.penaltyWeight.totalPower         = 0;    % Only considers power terms > 0
optimizationProperties.penaltyWeight.totalMechEnergy    = 0;
optimizationProperties.penaltyWeight.totalElecEnergy    = 0;
optimizationProperties.penaltyWeight.totalMechEnergyActive = 0;
optimizationProperties.penaltyWeight.averageEfficiency  = 0;    % Maximizes average efficiency for electrical motor operating point. *Note that efficiency maps are not yet validated.
optimizationProperties.penaltyWeight.maxTorque          = 0;
optimizationProperties.penaltyWeight.maxqdot            = 0;
optimizationProperties.penaltyWeight.maxPower           = 0;    % Only considers power terms > 0
optimizationProperties.penaltyWeight.antagonisticPower  = 0;    % Seeks to minimize antagonistic power which improves power quality
optimizationProperties.penaltyWeight.mechCoT            = 0;    % Mechanical cost of transport
optimizationProperties.penaltyWeight.maximumExtension   = true; % Large penalty incurred if leg extends beyond allowable amount
optimizationProperties.allowableExtension               = 0.9;  % Penalize extension above this ratio of total possible extension

%% OPTIMIZATION BOUNDS
% If upper and lower limit are set to 1, the parameter cannot change from
% the nominal value.
% Link lengths
optimizationProperties.bounds.lowerBoundMultiplier.hipLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.hipLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.thighLength = 0.8;
optimizationProperties.bounds.upperBoundMultiplier.thighLength = 1.2; 

optimizationProperties.bounds.lowerBoundMultiplier.shankLength = 0.8;
optimizationProperties.bounds.upperBoundMultiplier.shankLength = 1.2;

optimizationProperties.bounds.lowerBoundMultiplier.footLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.footLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength = 1;
optimizationProperties.bounds.upperBoundMultiplier.phalangesLength = 1;

optimizationProperties.bounds.lowerBoundMultiplier.hipOffset = 1;
optimizationProperties.bounds.upperBoundMultiplier.hipOffset = 1;

% Transmission gear ratio from actuator output to joint
optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.DFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.DFE = 1;

% Spring constant for springs in parallel with joint
optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.HAA = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.HAA = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.HFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.HFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.KFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.KFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.AFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.AFE = 1;

optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.DFE = 1;
optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.DFE = 1;

% Spring nominal position - limits specified as offset from the default
% nominal position
optimizationProperties.bounds.lowerBoundOffset.q0SpringJoint.HAA = 0;
optimizationProperties.bounds.upperBoundOffset.q0SpringJoint.HAA = 0;

optimizationProperties.bounds.lowerBoundOffset.q0SpringJoint.HFE = 0;
optimizationProperties.bounds.upperBoundOffset.q0SpringJoint.HFE = 0;

optimizationProperties.bounds.lowerBoundOffset.q0SpringJoint.KFE = -pi/4;
optimizationProperties.bounds.upperBoundOffset.q0SpringJoint.KFE =  pi/4;

optimizationProperties.bounds.lowerBoundOffset.q0SpringJoint.AFE = 0;
optimizationProperties.bounds.upperBoundOffset.q0SpringJoint.AFE = 0;

optimizationProperties.bounds.lowerBoundOffset.q0SpringJoint.DFE = 0;
optimizationProperties.bounds.upperBoundOffset.q0SpringJoint.DFE = 0;

% Spring for heuristic at AFE/DFE (for 3-4 link legs)
optimizationProperties.bounds.lowerBoundMultiplier.kTorsionalSpring = 1;
optimizationProperties.bounds.upperBoundMultiplier.kTorsionalSpring = 1;

optimizationProperties.bounds.lowerBoundMultiplier.thetaLiftoff_des = 1;
optimizationProperties.bounds.upperBoundMultiplier.thetaLiftoff_des = 1;

%% RUN SIMULATION
if recordCommandWindow
    diary commandWindowReadout
end

if ~optimizationProperties.runOptimization % if optimization turned off, set values to zero.
    optimizeLeg.LF = 0; optimizeLeg.RF = 0; optimizeLeg.LH = 0; optimizeLeg.RH = 0;
end

simulateSelectedTasks;
fprintf('Done.\n');
diary off