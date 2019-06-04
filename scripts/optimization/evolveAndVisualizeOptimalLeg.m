%% compute and visualize the optimal design
% takes task and robot selection to load corresponding towr motion data
% then computes motion of EE relative to hip, and evolves the link lengths
% to minimize sum of joint torques over full motion for this relative
% motion

% this function calls evolveOptimalLeg which runs the simulation
% runFastJointTorqueSim for each set of link lengths 

function [jointTorqueOpt, qOpt, qdotOpt, qdotdotOpt, rOpt, jointPowerOpt, linkLengths, penaltyMin, elapsedTime, elapsedTimePerFuncEval, output, linkMassOpt, totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg)
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    else selectFrontHind = 2;
end
%% initialize link length values
% link lengths in cm so that optimizer can consider only integer values 
initialLinkLengths(1) = quadruped.hip(selectFrontHind).length*100; 
initialLinkLengths(2) = quadruped.thigh(selectFrontHind).length*100;
initialLinkLengths(3) = quadruped.shank(selectFrontHind).length*100;
if (linkCount == 3) || (linkCount == 4)
    initialLinkLengths(4) = quadruped.foot(selectFrontHind).length*100;
end
if (linkCount == 4)
    initialLinkLengths(5) = quadruped.phalanges(selectFrontHind).length*100;
end

%% print statement
upperBnd = round(optimizationProperties.bounds.upperBoundMultiplier.*initialLinkLengths)/100;
lowerBnd = round(optimizationProperties.bounds.lowerBoundMultiplier.*initialLinkLengths)/100;
fprintf('\nLower bound on link lengths [m]:')
disp(lowerBnd);
fprintf('Upper bound on link lengths [m]:')
disp(upperBnd);

%% evolve optimal link lengths by running ga
tic;
[linkLengths, penaltyMin, output] = evolveOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg);
elapsedTime = toc;
elapsedTimePerFuncEval = elapsedTime/output.funccount;
fprintf('Optimized link lengths [m]:')
disp(linkLengths/100);

%% convert back from cm to m and save the final link lengths into quadruped
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;
if (linkCount == 3) || (linkCount == 4)
    quadruped.foot(selectFrontHind).length = linkLengths(4)/100;
end
if (linkCount ==4)
    quadruped.phlanges(selectFrontHind).length = linkLengths(5)/100;
end

% Update link mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1)/100;
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2)/100;
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3)/100;
if (linkCount == 3) || (linkCount == 4)
    quadruped.foot(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.foot(selectFrontHind).radius)^2 * linkLengths(4)/100;
end
if (linkCount ==4)
    quadruped.phalanges(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.phalanges(selectFrontHind).radius)^2 * linkLengths(5)/100;
end
%% visualize the optimized design
numberOfLoopRepetitions = 3;
viewVisualization = optimizationProperties.viz.viewVisualization;
%inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(l_hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(l_hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);

%% get joint torques of optimal design
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% get joint power for optimal design
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(1:end-2,1:end-1);

%% get link mass for optimal design
[linkMassOpt, ~, totalLinkMassOpt] = getLinkMass(tempLeg, EEselection, linkCount);

%% return joint data
linkLengths = linkLengths/100; %convert back to m for output
rOpt = tempLeg.(EEselection).r;
qOpt = tempLeg.(EEselection).q;
qdotOpt = tempLeg.(EEselection).qdot;
qdotdotOpt = tempLeg.(EEselection).qdotdot;
jointTorqueOpt = tempLeg.(EEselection).jointTorque;
jointPowerOpt = tempLeg.(EEselection).jointPower;
