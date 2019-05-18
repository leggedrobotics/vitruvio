%% compute and visualize the optimal design
% takes task and robot selection to load corresponding towr motion data
% then computes motion of EE relative to hip, and evolves the link lengths
% to minimize sum of joint torques over full motion for this relative
% motion

% this function calls evolveOptimalLeg which runs the simulation
% runFastJointTorqueSim for each set of link lengths 

function [jointTorqueOpt, qdotOpt, qdotdotOpt] = evolveAndVisualizeOptimalLeg(optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE)
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    else selectFrontHind = 2;
end
%% initialize link length values
% link lengths in cm
initialLinkLengths(1) = quadruped.hip(selectFrontHind).length*100; 
initialLinkLengths(2) = quadruped.thigh(selectFrontHind).length*100;
initialLinkLengths(3) = quadruped.shank(selectFrontHind).length*100;

%% print statement
upperBnd = round(optimizationProperties.upperBoundMultiplier.*initialLinkLengths)/100;
lowerBnd = round(optimizationProperties.lowerBoundMultiplier.*initialLinkLengths)/100;
fprintf('\nLower bound on link lengths [m]: %3.3f, %3.3f, %3.3f \n', lowerBnd);
fprintf('Upper bound on link lengths [m]: %3.3f, %3.3f, %3.3f \n\n', upperBnd);
    
%% evolve optimal link lengths by running ga
[linkLengths, penaltyMin] = evolveOptimalLeg(optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, EE, dt, jointCount, meanCyclicMotionHipEE);
fprintf('Optimized link lengths [m]: %3.3f, %3.3f, %3.3f \n\n', linkLengths/100);

%% convert back from cm to m and save the final link lengths
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;

%% visualize the optimized design
numberOfLoopRepetitions = 3;
viewVisualization = optimizationProperties.viewVisualization;
tempLeg.(EEselection).q = inverseKinematics(meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% get joint torques of optimal design
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);
tempLeg.(EEselection).jointTorque = getInverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE);

qdotOpt = tempLeg.(EEselection).qdot;
qdotdotOpt = tempLeg.(EEselection).qdotdot;
jointTorqueOpt = tempLeg.(EEselection).jointTorque;
