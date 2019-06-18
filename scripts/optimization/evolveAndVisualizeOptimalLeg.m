% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 

function [jointTorqueOpt, qOpt, qdotOpt, qdotdotOpt, rOpt, jointPowerOpt, linkLengthsOpt, hipAttachmentOffsetOpt, penaltyMin, elapsedTime, elapsedTimePerFuncEval, output, linkMassOpt, totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex)
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
%% initialize link length values
% link lengths in cm so that optimizer can consider only integer values 
initialLinkLengths(1) = quadruped.hip(selectFrontHind).length*100; 
initialLinkLengths(2) = quadruped.thigh(selectFrontHind).length*100;
initialLinkLengths(3) = quadruped.shank(selectFrontHind).length*100;
if (linkCount == 3) | (linkCount == 4)
    initialLinkLengths(4) = quadruped.foot(selectFrontHind).length*100;
end
if (linkCount == 4)
    initialLinkLengths(5) = quadruped.phalanges(selectFrontHind).length*100;
end
initialHipAttachmentOffset = hipAttachmentOffset*100;

%% print statement
% Ensure bounds are ordered correctly such that lower bound is always <=
% upper bound

upperBnd = [round(optimizationProperties.bounds.upperBoundMultiplier(1:linkCount+1).*initialLinkLengths)/100, round(optimizationProperties.bounds.upperBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)/100];
lowerBnd = [round(optimizationProperties.bounds.lowerBoundMultiplier(1:linkCount+1).*initialLinkLengths)/100, round(optimizationProperties.bounds.lowerBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)/100];

for i = 1:length(upperBnd)
    if (lowerBnd(i) > upperBnd(i))
        lowerBndTemp(i) = lowerBnd(i);
        lowerBnd(i) = upperBnd(i);
        upperBnd(i) = lowerBndTemp(i);
    end
end

fprintf('\nLower bound on link lengths [m]:')
disp(lowerBnd);
fprintf('Upper bound on link lengths [m]:')
disp(upperBnd);

%% evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(imposeJointLimits, heuristic, upperBnd, lowerBnd, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex);
elapsedTime = toc;
elapsedTimePerFuncEval = elapsedTime/output.funccount;
fprintf('Optimized leg design parameters [m]:')
disp(legDesignParameters/100);

%% convert back from cm to m and save the final link lengths into quadruped
linkLengths = legDesignParameters(1:linkCount+1)/100;
hipAttachmentOffset = legDesignParameters(linkCount+2)/100;
% Update link lengths, unit in meters
quadruped.hip(selectFrontHind).length = linkLengths(1);
quadruped.thigh(selectFrontHind).length = linkLengths(2);
quadruped.shank(selectFrontHind).length = linkLengths(3);
if (linkCount == 3) || (linkCount == 4)
    quadruped.foot(selectFrontHind).length = linkLengths(4);
end
if linkCount == 4
    quadruped.phalanges(selectFrontHind).length = linkLengths(5);
end

% Update link mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1);
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2);
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3);
if (linkCount == 3) | (linkCount == 4)
    quadruped.foot(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.foot(selectFrontHind).radius)^2 * linkLengths(4);
end
if (linkCount ==4)
    quadruped.phalanges(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.phalanges(selectFrontHind).radius)^2 * linkLengths(5);
end
%% visualize the optimized design
numberOfLoopRepetitions = optimizationProperties.viz.numberOfCyclesVisualized;
viewVisualization = optimizationProperties.viz.viewVisualization;

%% qAFE, qDFE torque based heuristic computation
% computation of parameters for first time step used to initialize the IK
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
    EE_force = Leg.(EEselection).force(1,1:3);
    rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(1,2); % rotation of body about inertial y
    qPrevious = qLiftoff.(EEselection);
    [springTorque.(EEselection), springDeformation.(EEselection)] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);      
else
    qLiftoff.(EEselection) = 0; % if the heuristic does not apply
end

% inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
% build rigid body model
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);

%% get joint torques of optimal design
% finite difference to compute qdot
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);
% inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% get joint power for optimal design
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);

%% get link mass for optimal design
[linkMassOpt, ~, totalLinkMassOpt] = getLinkMass(tempLeg, EEselection, linkCount);

%% return joint data
linkLengthsOpt = linkLengths;
hipAttachmentOffsetOpt = hipAttachmentOffset;
rOpt = tempLeg.(EEselection).r;
qOpt = tempLeg.(EEselection).q;
qdotOpt = tempLeg.(EEselection).qdot;
qdotdotOpt = tempLeg.(EEselection).qdotdot;
jointTorqueOpt = tempLeg.(EEselection).jointTorque;
jointPowerOpt = tempLeg.(EEselection).jointPower;