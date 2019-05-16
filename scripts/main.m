% main

clear;
close all;

%% Load task
% Select task and robot to be loaded
taskSelection = 'universalTrot'; % universalTrot, universalStairs, speedyGallop, speedyStairs, massivoWalk, massivoStairs, centaurWalk, centaurStairs, miniPronk
robotSelection = 'universal';
configSelection = 'M';
viewVisualization = true; 
viewPlots = true;

fprintf('Loading data for %s.\n', taskSelection);

% Get suggested removal ratio for cropping motion data to useful steady state motion
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

% Load motion and force data from .mat file
load(taskSelection);

%% Load corresponding robot parameters
fprintf('Getting quadruped properties.\n');
quadruped = getQuadrupedProperties(robotSelection);

%% Get the relative motion of the end effectors to the hips
fprintf('getting motion of end effectors relative to hip attachment points \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% Get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
fprintf('getting end effector liftoff and touchdown timings \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% Get the mean cyclic position and forces
fprintf('getting average relative motion of end effectors for one step \n');
[meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody);

%% Get reachable positions for plot
fprintf('getting range of motion dependent on link lengths and joint limits \n');
reachablePositions = getRangeofMotion(quadruped);

%% Plot data
if viewPlots
    fprintf('plotting data \n');
    plotMotionData;
end

%% Inverse kinematics to calculate joint angles for each leg joint
% these q0 give x config for universalStairs
% Final term is selectFrontHind 1 = front legs, 2 = hind legs

fprintf('getting joint angles from inverse kinematics \n');

EEselection = 'LF';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'LH';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'RF';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'RH';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);

%% Forward kinematics to get joint positions based on angles q solved in IK
% this validates results of IK

jointCount = 3; % not yet able to handle 4 joints
fprintf('getting joint positions from forward kinematics \n');
EEselection = 'LF';
r = getJointPositions(quadruped, q, jointCount, EEselection);

%% Visualize motion of single leg with rigid body model - now using different method to generate rigid body model

% fprintf('creating robot rigid body model and visualizing selected leg \n');
% numberOfLoopRepetitions = 1;
% 
% EEselection = 'LF';
% robot = robotSingleLegVisualization(quadruped, q, meanCyclicC_IBody, EE, meanCyclicMotionHipEE,EEselection, reachablePositions,numberOfLoopRepetitions);
% 
% EEselection = 'RF';
% robot = robotSingleLegVisualization(quadruped, q, meanCyclicC_IBody, EE, meanCyclicMotionHipEE,EEselection, reachablePositions,numberOfLoopRepetitions);
% 
% EEselection = 'LH';
% robot = robotSingleLegVisualization(quadruped, q, meanCyclicC_IBody, EE, meanCyclicMotionHipEE,EEselection, reachablePositions,numberOfLoopRepetitions);
% 
% EEselection = 'RH';
% robot = robotSingleLegVisualization(quadruped, q, meanCyclicC_IBody, EE, meanCyclicMotionHipEE,EEselection, reachablePositions,numberOfLoopRepetitions);


%% build robot model with configuration method - required for inverse dynamics solver
fprintf('creating robot rigid body model with configuration method \n');
numberOfLoopRepetitions = 1;

EEselection = 'LF';
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
EEselection = 'RF';
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
EEselection = 'LH';
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
EEselection = 'RH';
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
  
% the joint accelerations are then computed using finite difference
% seems to be a bug here for EE != LF 
fprintf('getting joint velocities and accelerations \n');

EEselection = 'LF';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt);
EEselection = 'RF';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt);
EEselection = 'LH';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt);
EEselection = 'RH';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt);

%% Get joint torques using inverse dynamics

fprintf('getting joint torques from inverse dynamics \n');
EEselection = 'LF';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'RF';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'LH';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'RH';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);

