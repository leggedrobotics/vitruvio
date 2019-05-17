%% compute and visualize the optimal design
% takes task and robot selection to load corresponding towr motion data
% then computes motion of EE relative to hip, and evolves the link lengths
% to minimize sum of joint torques over full motion for this relative
% motion

% this function calls evolveOptimalLeg which runs the simulation
% runFastJointTorqueSim for each set of link lengths 

function [jointTorqueOpt, qOptVel, qOptAccel] = evolveAndVisualizeOptimalLeg(viewVisualization, upperBoundMultiplier, lowerBoundMultiplier, maxGenerations, populationSize, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE)
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
disp('Dimensions given in cm')
initialLinkLengths
upperBnd = round(upperBoundMultiplier*initialLinkLengths)
lowerBnd = round(lowerBoundMultiplier*initialLinkLengths)
            
%% evolve optimal link lengths by running ga
[linkLengths, penaltyMin] = evolveOptimalLeg(maxGenerations, populationSize, initialLinkLengths, upperBoundMultiplier, lowerBoundMultiplier, taskSelection, quadruped, configSelection, EEselection, EE, dt, jointCount, meanCyclicMotionHipEE)

%% convert back from cm to m and save the final link lengths
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;

%% visualize the optimized design
numberOfLoopRepetitions = 3;

qOpt.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, qOpt, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% get joint torques of optimal design
[qOpt.(EEselection).angVel, qOpt.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, qOpt, quadruped, dt);
jointTorqueOpt = getInverseDynamics(EEselection, qOpt, meanCyclicMotionHipEE, robotConfig, config);

qOptVel = qOpt.(EEselection).angVel; 
qOptAccel = qOpt.(EEselection).angAccel; 