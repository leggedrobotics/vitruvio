% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 

function [jointTorqueOpt, qOpt, qdotOpt, qdotdotOpt, rOpt, jointPowerOpt, linkLengthsOpt, hipAttachmentOffsetOpt, penaltyMin, elapsedTime, elapsedTimePerFuncEval, output, linkMassOpt, totalLinkMassOpt, deltaqMaxOpt, qdotMaxOpt, jointTorqueMaxOpt, jointPowerMaxOpt, mechEnergyOpt, mechEnergyPerCycleOpt, mechEnergyPerCycleTotalOpt, elecEnergyOpt, elecEnergyPerCycleOpt, elecEnergyPerCycleTotalOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction)

if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
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
initialHipAttachmentOffset = hipAttachmentOffset*100;

%% print statement
% Ensure bounds are ordered correctly such that lower bound is always <=
% upper bound. Bounds in cm.
upperBnd = [round(optimizationProperties.bounds.upperBoundMultiplier(1:linkCount+1).*initialLinkLengths), round(optimizationProperties.bounds.upperBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)];
lowerBnd = [round(optimizationProperties.bounds.lowerBoundMultiplier(1:linkCount+1).*initialLinkLengths), round(optimizationProperties.bounds.lowerBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)];

for i = 1:length(upperBnd)
    if (lowerBnd(i) > upperBnd(i))
        lowerBndTemp(i) = lowerBnd(i);
        lowerBnd(i) = upperBnd(i);
        upperBnd(i) = lowerBndTemp(i);
    end
end

fprintf('\nLower bound on link lengths [m]:')
disp(lowerBnd/100);
fprintf('Upper bound on link lengths [m]:')
disp(upperBnd/100);

%% evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBnd, lowerBnd, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction);
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
if (linkCount == 3) || (linkCount == 4)
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
    [qLiftoff.(EEselection{1})] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
    EE_force = Leg.(EEselection).force(1,1:3);
    rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
    qPrevious = qLiftoff.(EEselection);
    [springTorque.(EEselection), springDeformation.(EEselection)] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);      
else
    qLiftoff.(EEselection) = 0; % if the heuristic does not apply
end

%% inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% build rigid body model
tempLeg.base = Leg.base;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody, dataExtraction);

%% get joint velocities and accelerations using finite difference
% finite difference to compute qdot
tempLeg.time = Leg.time;
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg);
tempLeg.(EEselection).q = tempLeg.(EEselection).q(1:end-2,:); % no longer need the two additional points for position after solving for joint speed and acceleration

%% inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% get joint power for optimal design
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);

%% get link mass for optimal design
[linkMassOpt, ~, totalLinkMassOpt] = getLinkMass(tempLeg, EEselection, linkCount);

%% get maximum joint states
[deltaqMaxOpt, qdotMaxOpt, jointTorqueMaxOpt, jointPowerMaxOpt]  = getMaximumJointStates(tempLeg, EEselection);    

%% Get electrical power and efficiency at each operating point
[tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

%% Get meta parameters
[tempLeg.(EEselection).mechEnergyOpt, tempLeg.metaParameters.mechEnergyPerCycleOpt.(EEselection), tempLeg.(EEselection).elecEnergyOpt, tempLeg.metaParameters.elecEnergyPerCycleOpt.(EEselection)]  = computeEnergyConsumption(tempLeg, EEselection, dt);
 tempLeg.metaParameters.mechEnergyPerCycleTotalOpt.(EEselection) = tempLeg.metaParameters.mechEnergyPerCycleOpt.(EEselection) + sum(tempLeg.metaParameters.mechEnergyPerCycleOpt.(EEselection));
 tempLeg.metaParameters.elecEnergyPerCycleTotalOpt.(EEselection) = tempLeg.metaParameters.elecEnergyPerCycleOpt.(EEselection) + sum(tempLeg.metaParameters.elecEnergyPerCycleOpt.(EEselection));    


%% return joint data
linkLengthsOpt = linkLengths;
hipAttachmentOffsetOpt = hipAttachmentOffset;
rOpt = tempLeg.(EEselection).r;
qOpt = tempLeg.(EEselection).q;
qdotOpt = tempLeg.(EEselection).qdot;
qdotdotOpt = tempLeg.(EEselection).qdotdot;
jointTorqueOpt = tempLeg.(EEselection).jointTorque;
jointPowerOpt = tempLeg.(EEselection).jointPower;
mechEnergyOpt = tempLeg.(EEselection).mechEnergyOpt;
mechEnergyPerCycleOpt = tempLeg.metaParameters.mechEnergyPerCycleOpt.(EEselection);
mechEnergyPerCycleTotalOpt = tempLeg.metaParameters.mechEnergyPerCycleTotalOpt.(EEselection);
elecEnergyOpt = tempLeg.(EEselection).elecEnergyOpt;
elecEnergyPerCycleOpt = tempLeg.metaParameters.elecEnergyPerCycleOpt.(EEselection);
elecEnergyPerCycleTotalOpt = tempLeg.metaParameters.elecEnergyPerCycleTotalOpt.(EEselection);