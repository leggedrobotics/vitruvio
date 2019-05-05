% main

clear all
close all

%% select task and robot to be loaded
taskSelection = 'speedyGallop';
robotSelection = 'speedy';

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
[relativeMotionHipEE, IF_hip] = getRelativeMotionEEHips(quat, quadruped, base, EE);

%% get the liftoff and touchdown timings for each end effector
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% get the mean cyclic position and forces
[meanCyclicMotionHipEE, cyclicMotionHipEE, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount);

%% plot data
[reachablePositionsFront reachablePositionsHind] = getRangeofMotion(quadruped);
plotMotionData;

%% get Jacobian
% [J_P, C_HEE, r_H_HEE]  = jointToPosJac(q, quadruped);

%% Inverse kinematics
q0 = [0 -pi/4 pi/2 0];
% Final term is selectFrontHind 1 = front legs, 2 = hind legs

q.LF = inverseKinematics(meanCyclicMotionHipEE.LF.position, q0, quadruped, 1);
q.LH = inverseKinematics(meanCyclicMotionHipEE.LF.position, q0, quadruped, 2);
q.RF = inverseKinematics(meanCyclicMotionHipEE.LF.position, q0, quadruped, 1);
q.RH = inverseKinematics(meanCyclicMotionHipEE.LF.position, q0, quadruped, 2);
