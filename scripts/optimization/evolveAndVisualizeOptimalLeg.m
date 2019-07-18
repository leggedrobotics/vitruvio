% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 
function [jointTorqueOpt, qOpt, qdotOpt, qdotdotOpt, rOpt, jointPowerOpt, linkLengthsOpt, hipAttachmentOffsetOpt, penaltyMin, elapsedTime, elapsedTimePerFuncEval, output, linkMassOpt, totalLinkMassOpt, deltaqMaxOpt, qdotMaxOpt, jointTorqueMaxOpt, jointPowerMaxOpt, mechEnergyOpt, mechEnergyPerCycleOpt, mechEnergyPerCycleTotalOpt, elecEnergyOpt, elecEnergyPerCycleOpt, elecEnergyPerCycleTotalOpt, elecPowerOpt, operatingPointEfficiencyOpt, operatingPointEfficiencyMeanOpt, springOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, robotProperties, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction)

if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
%% initialize link length values
% link lengths in cm so that optimizer can consider only integer values 
initialLinkLengths(1) = robotProperties.hip(selectFrontHind).length*100; 
initialLinkLengths(2) = robotProperties.thigh(selectFrontHind).length*100;
initialLinkLengths(3) = robotProperties.shank(selectFrontHind).length*100;
if (linkCount == 3) || (linkCount == 4)
    initialLinkLengths(4) = robotProperties.foot(selectFrontHind).length*100;
end
if (linkCount == 4)
    initialLinkLengths(5) = robotProperties.phalanges(selectFrontHind).length*100;
end
initialHipAttachmentOffset = hipAttachmentOffset*100;

%% Input the upper and lower bounds for the optimization
% upperBound = [round(optimizationProperties.bounds.upperBoundMultiplier(1:linkCount+1).*initialLinkLengths), round(optimizationProperties.bounds.upperBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)];
% lowerBound = [round(optimizationProperties.bounds.lowerBoundMultiplier(1:linkCount+1).*initialLinkLengths), round(optimizationProperties.bounds.lowerBoundMultiplier(linkCount+2)*initialHipAttachmentOffset)];

upperBound = [optimizationProperties.bounds.upperBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.upperBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.upperBoundMultiplier.shankLength*initialLinkLengths(3)];

lowerBound = [optimizationProperties.bounds.lowerBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.lowerBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.lowerBoundMultiplier.shankLength*initialLinkLengths(3)];  
          
if linkCount == 2 % hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.hipOffset*initialHipAttachmentOffset];
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset];   
end
          
if linkCount == 3 % foot length and hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.upperBoundMultiplier.hipOffset*initialHipAttachmentOffset];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset];    
end

if linkCount == 4 % phalanges length and hip attachment offset
    % Bounds on phalanges link length
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...    
                  optimizationProperties.bounds.upperBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset];

    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset]; 
end
              
if linkCount > 2 && heuristic.torqueAngle.apply % Bounds on torsional spring at final joint
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.kTorsionalSpring.*heuristic.torqueAngle.kTorsionalSpring, ...
                  optimizationProperties.bounds.upperBoundMultiplier.thetaLiftoff_des.*heuristic.torqueAngle.thetaLiftoff_des];
    
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.kTorsionalSpring.*heuristic.torqueAngle.kTorsionalSpring, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.thetaLiftoff_des.*heuristic.torqueAngle.thetaLiftoff_des];  
end

% Ensure bounds ordered corrrectly such that lower bound <= upper bound.
for i = 1:length(upperBound)
    if (lowerBound(i) > upperBound(i))
        lowerBoundTemp(i) = lowerBound(i);
        lowerBound(i) = upperBound(i);
        upperBound(i) = lowerBoundTemp(i);
    end
end

fprintf('\nLower bound on design parameters:')
disp(lowerBound/100);
fprintf('Upper bound on design parameters:')
disp(upperBound/100);

%% evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction);
elapsedTime = toc;
elapsedTimePerFuncEval = elapsedTime/output.funccount;
fprintf('Optimized leg design parameters :')
disp(legDesignParameters);


%% convert back from cm to m and save the final link lengths into quadruped
linkLengths = legDesignParameters(1:linkCount+1)/100;
hipAttachmentOffset = legDesignParameters(linkCount+2)/100;
if linkCount > 2 && heuristic.torqueAngle.apply
    springOpt.kTorsionalSpring = legDesignParameters(end-1);
    springOpt.thetaLiftoff_des = legDesignParameters(end);
else 
    springOpt.kTorsionalSpring = [];
    springOpt.thetaLiftoff_des = [];
end
% Update link lengths, unit in meters
robotProperties.hip(selectFrontHind).length = linkLengths(1);
robotProperties.thigh(selectFrontHind).length = linkLengths(2);
robotProperties.shank(selectFrontHind).length = linkLengths(3);
if (linkCount == 3) || (linkCount == 4)
    robotProperties.foot(selectFrontHind).length = linkLengths(4);
end
if linkCount == 4
    robotProperties.phalanges(selectFrontHind).length = linkLengths(5);
end

% Update link mass with assumption of constant density cylinder
robotProperties.hip(selectFrontHind).mass = robotProperties.legDensity.hip(selectFrontHind) * pi*(robotProperties.hip(selectFrontHind).radius)^2   * linkLengths(1);
robotProperties.thigh(selectFrontHind).mass = robotProperties.legDensity.thigh(selectFrontHind) * pi*(robotProperties.thigh(selectFrontHind).radius)^2 * linkLengths(2);
robotProperties.shank(selectFrontHind).mass = robotProperties.legDensity.shank(selectFrontHind) * pi*(robotProperties.shank(selectFrontHind).radius)^2 * linkLengths(3);
if (linkCount == 3) || (linkCount == 4)
    robotProperties.foot(selectFrontHind).mass = robotProperties.legDensity.foot(selectFrontHind) * pi*(robotProperties.foot(selectFrontHind).radius)^2 * linkLengths(4);
end
if (linkCount ==4)
    robotProperties.phalanges(selectFrontHind).mass = robotProperties.legDensity.phalanges(selectFrontHind) * pi*(robotProperties.phalanges(selectFrontHind).radius)^2 * linkLengths(5);
end
%% visualize the optimized design
numberOfLoopRepetitions = optimizationProperties.viz.numberOfCyclesVisualized;
viewVisualization = optimizationProperties.viz.viewVisualization;

%% qAFE, qDFE torque based heuristic computation
% computation of parameters for first time step used to initialize the IK
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    heuristic.torqueAngle.kTorsionalSpring = springOpt.kTorsionalSpring;
    heuristic.torqueAngle.thetaLiftoff_des = springOpt.thetaLiftoff_des;
    [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, configSelection, hipParalleltoBody);
    EE_force = Leg.(EEselection).force(1,1:3);
    rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
    qPrevious = qLiftoff.(EEselection);
    [springTorque.(EEselection), springDeformation.(EEselection)] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, robotProperties, EEselection, hipParalleltoBody);      
else
    qLiftoff.(EEselection) = 0; % if the heuristic does not apply
end

%% inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% build rigid body model
tempLeg.base = Leg.base;
optimized = true;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointDirectly, hipAttachmentOffset, linkCount, robotProperties, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody, dataExtraction, optimized);

%% get joint velocities and accelerations using finite difference
% finite difference to compute qdot
tempLeg.time = Leg.time;
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg, dt);
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
[tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(tempLeg.(EEselection).jointPower, tempLeg.(EEselection).electricalPower, dt);
 tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.mechEnergyPerCycle.(EEselection)(end,:));
 tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)(end,:));  

%% return joint data
linkLengthsOpt = linkLengths;
hipAttachmentOffsetOpt = hipAttachmentOffset;
rOpt = tempLeg.(EEselection).r;
qOpt = tempLeg.(EEselection).q;
qdotOpt = tempLeg.(EEselection).qdot;
qdotdotOpt = tempLeg.(EEselection).qdotdot;
jointTorqueOpt = tempLeg.(EEselection).jointTorque;
jointPowerOpt = tempLeg.(EEselection).jointPower;
mechEnergyOpt = tempLeg.(EEselection).mechEnergy;
mechEnergyPerCycleOpt = tempLeg.metaParameters.mechEnergyPerCycle.(EEselection);
mechEnergyPerCycleTotalOpt = tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection);
elecEnergyOpt = tempLeg.(EEselection).elecEnergy;
elecEnergyPerCycleOpt = tempLeg.metaParameters.elecEnergyPerCycle.(EEselection);
elecEnergyPerCycleTotalOpt = tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection);
elecPowerOpt  = tempLeg.(EEselection).electricalPower;
operatingPointEfficiencyOpt = tempLeg.(EEselection).operatingPointEfficiency;
operatingPointEfficiencyMeanOpt = mean(tempLeg.(EEselection).operatingPointEfficiency);