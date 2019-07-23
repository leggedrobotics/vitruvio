% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 
function optimizationResults = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, robotProperties, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames, saveFiguresToPDF)
if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
%% Initialize link length values
initialLinkLengths(1) = robotProperties.hip(selectFrontHind).length; 
initialLinkLengths(2) = robotProperties.thigh(selectFrontHind).length;
initialLinkLengths(3) = robotProperties.shank(selectFrontHind).length;
if (linkCount == 3) || (linkCount == 4)
    initialLinkLengths(4) = robotProperties.foot(selectFrontHind).length;
end
if (linkCount == 4)
    initialLinkLengths(5) = robotProperties.phalanges(selectFrontHind).length;
end
initialHipAttachmentOffset = hipAttachmentOffset;

%% Input the upper and lower bounds for the optimization
% Order of bounds is as follows:
% [link lengths, hip offset x from nom position, transmission gear ratios,
% torsional spring constant, liftoff angle]

upperBound = [optimizationProperties.bounds.upperBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.upperBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.upperBoundMultiplier.shankLength*initialLinkLengths(3)];

lowerBound = [optimizationProperties.bounds.lowerBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.lowerBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.lowerBoundMultiplier.shankLength*initialLinkLengths(3)];  
          
if linkCount == 2 % hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE];
end
          
if linkCount == 3 % foot length and hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.upperBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE];    
end

if linkCount == 4 % phalanges length and hip attachment offset
    % Bounds on phalanges link length
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...    
                  optimizationProperties.bounds.upperBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.upperBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.DFE];

    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.hipOffset*initialHipAttachmentOffset, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.DFE];
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

% Get design parameter names
if linkCount == 2 % hip attachment offset
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'nominal hip position translation x direction', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio'};
                        
elseif linkCount == 3
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'foot length', ...
                            'nominal hip position translation x direction', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio', ... 
                            'AFE transmission gear ratio'};
                        
elseif linkCount == 4 
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'foot length', ...
                            'phalanges length', ...
                            'nominal hip position translation x direction', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio', ... 
                            'AFE transmission gear ratio', ...
                            'DFE transmission gear ratio'};
end
                        
if linkCount > 2 && heuristic.torqueAngle.apply
    designParameterNames = [designParameterNames, ...
                            {'torsional spring constant', ...
                            'liftoff angle'}];
end

for i = 1:length(designParameterNames)
    fprintf('Bounds on %s [%3.2f, %3.2f] \n', designParameterNames{i}, lowerBound(i), upperBound(i))
end

%% Evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames);
optimizationResults.elapsedTime = toc;
optimizationResults.elapsedTimePerFuncEval = optimizationResults.elapsedTime/output.funccount;
fprintf('Optimized leg design parameters :')
disp(legDesignParameters);

%% Save optimized parameters
linkLengths = legDesignParameters(1:linkCount+1);
hipAttachmentOffset = legDesignParameters(linkCount+2);
transmissionGearRatio = legDesignParameters(linkCount+3:2*linkCount+3);

if linkCount > 2 && heuristic.torqueAngle.apply
    springOpt.kTorsionalSpring = legDesignParameters(end-1);
    springOpt.thetaLiftoff_des = legDesignParameters(end);
else 
    springOpt.kTorsionalSpring = [];
    springOpt.thetaLiftoff_des = [];
end

robotProperties.hip(selectFrontHind).length = linkLengths(1);
robotProperties.thigh(selectFrontHind).length = linkLengths(2);
robotProperties.shank(selectFrontHind).length = linkLengths(3);
if (linkCount == 3) || (linkCount == 4)
    robotProperties.foot(selectFrontHind).length = linkLengths(4);
end
if linkCount == 4
    robotProperties.phalanges(selectFrontHind).length = linkLengths(5);
end

for i = 1:linkCount+1
    robotProperties.transmissionGearRatioOpt.(jointNames(i,:))(selectFrontHind) = transmissionGearRatio(i);
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

%% Visualize the optimized design
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

%% Inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% Build rigid body model
tempLeg.base = Leg.base;
optimized = true;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointDirectly, hipAttachmentOffset, linkCount, robotProperties, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody, dataExtraction, optimized, saveFiguresToPDF);

%% Get joint velocities and accelerations using finite difference
% finite difference to compute qdot
tempLeg.time = Leg.time;
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg, dt);
tempLeg.(EEselection).q = tempLeg.(EEselection).q(1:end-2,:); % no longer need the two additional points for position after solving for joint speed and acceleration
% Smoothen result using moving average
for j = 1:length(Leg.(EEselection).qdot(1,:))
    tempLeg.(EEselection).qdot(:,j) = smooth(tempLeg.(EEselection).qdot(:,j));
    tempLeg.(EEselection).qdotdot(:,j) = smooth(tempLeg.(EEselection).qdotdot(:,j));
end
%% Inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);
% Smoothen result using moving average
for j = 1:length(Leg.(EEselection).jointTorque(1,:))
    tempLeg.(EEselection).jointTorque(:,j) = smooth(tempLeg.(EEselection).jointTorque(:,j));
end

%% Get joint power for optimal design
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);

%% Get actuator torque and speed as result of gearing between actuator and joint
% This is the required output torque and speed from the actuator to produce
% the joint torque and speed.
for j = 1:linkCount+1
    % gear ratio = actuator speed / joint speed = joint torque/
    % actuator torque
    transmissionGearRatio = robotProperties.transmissionGearRatioOpt.(jointNames(j,:))(selectFrontHind);
    tempLeg.(EEselection).actuatorq(:,j)      = transmissionGearRatio * tempLeg.(EEselection).q(:,j);
    tempLeg.(EEselection).actuatorqdot(:,j)   = transmissionGearRatio * tempLeg.(EEselection).qdot(:,j);
    tempLeg.(EEselection).actuatorTorque(:,j) = (1/transmissionGearRatio) * tempLeg.(EEselection).jointTorque(:,j);
end

%% Get link mass for optimal design
[linkMassOpt, ~, totalLinkMassOpt] = getLinkMass(tempLeg, EEselection, linkCount);

%% Get maximum joint states
[deltaqMaxOpt, qdotMaxOpt, jointTorqueMaxOpt, jointPowerMaxOpt]  = getMaximumJointStates(tempLeg, EEselection);    
% Maximum actuator states
for j = 1:linkCount+1
    % gear ratio = actuator speed / joint speed = joint torque/
    % actuator torque
    transmissionGearRatio(j) = robotProperties.transmissionGearRatioOpt.(jointNames(j,:))(selectFrontHind);
    actuatordeltaqMaxOpt(j)  = transmissionGearRatio(j) * deltaqMaxOpt(j);
    actuatorqdotMaxOpt(j)    = transmissionGearRatio(j) * qdotMaxOpt(j);
    actuatorTorqueMaxOpt(j)  = (1/transmissionGearRatio(j)) * jointTorqueMaxOpt(j);
end

%% Get electrical power and efficiency at each operating point
[tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

%% Get meta parameters
[tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(tempLeg.(EEselection).jointPower, tempLeg.(EEselection).electricalPower, dt);
 tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.mechEnergyPerCycle.(EEselection)(end,:));
 tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)(end,:));  

%% Return the results of the optimization
optimizationResults.linkLengthsOpt = linkLengths;
optimizationResults.hipOffsetOpt = hipAttachmentOffset;
optimizationResults.transmissionGearRatioOpt = transmissionGearRatio;
optimizationResults.springOpt.kTorsionalSpring = springOpt.kTorsionalSpring;
optimizationResults.springOpt.thetaLiftoff_des = springOpt.thetaLiftoff_des;
optimizationResults.rOpt = tempLeg.(EEselection).r;
optimizationResults.qOpt = tempLeg.(EEselection).q;
optimizationResults.qdotOpt = tempLeg.(EEselection).qdot;
optimizationResults.qdotdotOpt = tempLeg.(EEselection).qdotdot;
optimizationResults.jointTorqueOpt = tempLeg.(EEselection).jointTorque;
optimizationResults.jointPowerOpt = tempLeg.(EEselection).jointPower;
optimizationResults.mechEnergyOpt = tempLeg.(EEselection).mechEnergy;
optimizationResults.mechEnergyPerCycleOpt = tempLeg.metaParameters.mechEnergyPerCycle.(EEselection);
optimizationResults.mechEnergyPerCycleTotalOpt = tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection);
optimizationResults.elecEnergyOpt = tempLeg.(EEselection).elecEnergy;
optimizationResults.elecEnergyPerCycleOpt = tempLeg.metaParameters.elecEnergyPerCycle.(EEselection);
optimizationResults.elecEnergyPerCycleTotalOpt = tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection);
optimizationResults.elecPowerOpt  = tempLeg.(EEselection).electricalPower;
optimizationResults.operatingPointEfficiencyOpt = tempLeg.(EEselection).operatingPointEfficiency;
optimizationResults.operatingPointEfficiencyMeanOpt = mean(tempLeg.(EEselection).operatingPointEfficiency);
optimizationResults.actuatorqOpt = tempLeg.(EEselection).actuatorq;
optimizationResults.actuatorqdotOpt = tempLeg.(EEselection).actuatorqdot;
optimizationResults.actuatorTorqueOpt = tempLeg.(EEselection).actuatorTorque;
optimizationResults.penaltyMinOpt = penaltyMin;
optimizationResults.linkMassOpt = linkMassOpt;
optimizationResults.totalLinkMassOpt = totalLinkMassOpt;
optimizationResults.deltaqMaxOpt = deltaqMaxOpt;
optimizationResults.qdotMaxOpt = qdotMaxOpt;
optimizationResults.jointTorqueMaxOpt = jointTorqueMaxOpt;
optimizationResults.jointPowerMaxOpt = jointPowerMaxOpt;
optimizationResults.actuatordeltaqMaxOpt = actuatordeltaqMaxOpt;
optimizationResults.actuatorqdotMaxOpt = actuatorqdotMaxOpt;
optimizationResults.actuatorTorqueMaxOpt = actuatorTorqueMaxOpt;
optimizationResults.gaSettings = output;