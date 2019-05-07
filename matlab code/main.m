% main

clear all
close all

%% select task and robot to be loaded
taskSelection = 'universalTrot';
robotSelection = 'universal';

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

quadruped = getQuadrupedProperties(robotSelection);

%% get the relative motion of the end effectors to the hips
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% get the mean cyclic position and forces
[meanCyclicMotionHipEE, cyclicMotionHipEE, cyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody);

%% get reachable positions for plot
reachablePositions = getRangeofMotion(quadruped);

%% plot data
% plotMotionData;

%% Inverse kinematics to calculate joint angles for each leg joint
q0 = [0 -pi/4 -pi/2 0];
% Final term is selectFrontHind 1 = front legs, 2 = hind legs

q.LF = inverseKinematics(meanCyclicMotionHipEE.LF.position, q0, quadruped, 1);
q.LH = inverseKinematics(meanCyclicMotionHipEE.LH.position, q0, quadruped, 2);
q.RF = inverseKinematics(meanCyclicMotionHipEE.RF.position, q0, quadruped, 1);
q.RH = inverseKinematics(meanCyclicMotionHipEE.RH.position, q0, quadruped, 2);

%% Forward kinematics to get joint positions based on angles q solved in IK
jointCount = 3; % not yet able to handle 4 joints
r = getJointPositions(quadruped, q, jointCount);

%% Visualize motion of single leg with rigid body model
% EESlection: 1=LF, 2=LH, 3=RF, 4=RH
EESelection = 3;
% robotSingleLegVisualization(quadruped, q, C_IBody, EE, meanCyclicMotionHipEE,EESelection, reachablePositions)

%% get Jacobian for inverse dynamics
selectFrontHind = 1; 
  [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34] = jointToPosJac(q.LF, quadruped, selectFrontHind);

