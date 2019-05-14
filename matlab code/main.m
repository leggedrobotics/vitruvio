% main

clear all
close all

%% select task and robot to be loaded
taskSelection = 'speedyStairs';
robotSelection = 'speedy';
configSelection = 'M';
viewVisualization = 1; % 1 is on
viewPlots = 1;

fprintf('loading data for %s \n', taskSelection);

%% get suggested removal ratio for cropping motion data to useful steady state motion

[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

%% load motion and force data
% universalTrot
% universalStairs
% speedyGallop
% speedyStairs
% massivoWalk
% massivoStairs
% centaurWalk
% centaurStairs
% miniPronk

load(taskSelection);

%% load corresponding robot parameters
% universal
% speedy
% massivo
% centaur
% mini
fprintf('getting quadruped properties \n');
quadruped = getQuadrupedProperties(robotSelection);
%% get the relative motion of the end effectors to the hips
fprintf('getting motion of end effectors relative to hip attachment points \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
fprintf('getting end effector liftoff and touchdown timings \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% get the mean cyclic position and forces
fprintf('getting average relative motion of end effectors for one step \n');
[meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody);

%% get reachable positions for plot
fprintf('getting range of motion dependent on link lengths and joint limits \n');
reachablePositions = getRangeofMotion(quadruped);

%% plot data
if viewPlots == 1
    fprintf('plotting data \n');
    plotMotionData;
end

%% Inverse kinematics to calculate joint angles for each leg joint
% these q0 give x config for universalStairs
% Final term is selectFrontHind 1 = front legs, 2 = hind legs

fprintf('getting joint angles from inverse kinematics \n');

EEselection = 'LF';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.LF.position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'LH';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.LH.position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'RF';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.RF.position, quadruped, EEselection, taskSelection, configSelection);

EEselection = 'RH';
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.RH.position, quadruped, EEselection, taskSelection, configSelection);

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

%% get joint velocities with inverse(Jacobian)* EE.velocity
  
% the joint accelerations are then computed using finite difference
fprintf('getting joint velocities and accelerations \n');

EEselection = 'LF';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EE, meanCyclicMotionHipEE, q, quadruped, 1, dt, EEselection);
EEselection = 'RF';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EE, meanCyclicMotionHipEE, q, quadruped, 1, dt, EEselection);
EEselection = 'LH';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EE, meanCyclicMotionHipEE, q, quadruped, 1, dt, EEselection);
EEselection = 'RH';
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EE, meanCyclicMotionHipEE, q, quadruped, 1, dt, EEselection);

%% get joint torques using inverse dynamics

fprintf('getting joint torques from inverse dynamics \n');
EEselection = 'LF';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'RF';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'LH';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);
EEselection = 'RH';
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);

