% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 
function optimizationResults = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointDirectly, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, robotProperties, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, q0SpringJoint)
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
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE];
end
          
if linkCount == 3 % foot length and hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength*initialLinkLengths(4), ...
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
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.DFE];

    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
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

% Extend end of bounds array with the bounds on spring constants for
% springs in parallel with the joints
if springInParallelWithJoints
    for i = 1:linkCount+1  
        upperBound(end+1) = optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.(jointNames(i,:))*kSpringJoint.(EEselection)(i);
        lowerBound(end+1) = optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.(jointNames(i,:))*kSpringJoint.(EEselection)(i);
    end
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
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio'};
                        
elseif linkCount == 3
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'foot length', ...
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
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio', ... 
                            'AFE transmission gear ratio', ...
                            'DFE transmission gear ratio'};
end

if springInParallelWithJoints
    designParameterNames = [designParameterNames, ...
                            {'torsional spring constant at HAA'}, ...
                            {'torsional spring constant at HFE'}, ...
                            {'torsional spring constant at KFE'}];
    if linkCount > 2
        designParameterNames = [designParameterNames, ...
                                {'torsional spring constant at AFE'}]; 
    end
    if linkCount > 3
        designParameterNames = [designParameterNames, ...
                                {'torsional spring constant at DFE'}]; 
    end
end
        

if linkCount > 2 && heuristic.torqueAngle.apply
    designParameterNames = [designParameterNames, ...
                            {'torsional spring constant AFE/DFE', ...
                            'liftoff angle'}];
end

for i = 1:length(designParameterNames)
    fprintf('Bounds on %s [%3.2f, %3.2f] \n', designParameterNames{i}, lowerBound(i), upperBound(i))
end

%% Evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames, springInParallelWithJoints, kSpringJoint, q0SpringJoint);
optimizationResults.elapsedTime = toc;
optimizationResults.elapsedTimePerFuncEval = optimizationResults.elapsedTime/output.funccount;
fprintf('Optimized leg design parameters :')
disp(legDesignParameters);

%% Save optimized parameters
tempLeg.base = Leg.base;
tempLeg.(EEselection).force = Leg.(EEselection).force;
tempLeg.basicProperties = Leg.basicProperties;

jointCount = linkCount+1;
linkLengths = legDesignParameters(1:jointCount);
transmissionGearRatio = legDesignParameters(jointCount+1:2*jointCount);
if springInParallelWithJoints
    kSpringJointOpt.(EEselection) = legDesignParameters(2*jointCount+1:3*jointCount);
else
    kSpringJointOpt.(EEselection) = [0 0 0 0 0];
end
    
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

tempLeg.robotProperties = robotProperties;

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
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(tempLeg, heuristic, qLiftoff, meanCyclicMotionHipEE, EEselection);

%% Build rigid body model
tempLeg.base = Leg.base;
% Create rigid body model for swing and stance. The only difference is the
% presence of the gravity term.
gravitySwing  = [0 0 -9.81]; % During swing phase, apply gravity
gravityStance = [0 0 0]; % Do not apply gravity term in stance as the end-effector forces already include the weight of the legs and additional gravity here would be double counting the gravitational weight on the joints
tempLeg.(EEselection).rigidBodyModelSwing  = buildRobotRigidBodyModel(gravitySwing, actuatorProperties, actuateJointDirectly, linkCount, robotProperties, tempLeg, meanCyclicMotionHipEE, EEselection, hipParalleltoBody);
tempLeg.(EEselection).rigidBodyModelStance = buildRobotRigidBodyModel(gravityStance, actuatorProperties, actuateJointDirectly, linkCount, robotProperties, tempLeg, meanCyclicMotionHipEE, EEselection, hipParalleltoBody);

%% Get joint velocities and accelerations using finite difference
% finite difference to compute qdot
tempLeg.time = Leg.time;
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg, dt);
tempLeg.(EEselection).q = tempLeg.(EEselection).q(1:end-2,:); % no longer need the two additional points for position after solving for joint speed and acceleration
% Smoothen result using moving average
% for j = 1:length(Leg.(EEselection).qdot(1,:))
%     tempLeg.(EEselection).qdot(:,j) = smooth(tempLeg.(EEselection).qdot(:,j));
%     tempLeg.(EEselection).qdotdot(:,j) = smooth(tempLeg.(EEselection).qdotdot(:,j));
% end

%% Inverse dynamics
externalForce = Leg.(EEselection).force(:,1:3);
tempLeg.(EEselection).jointTorqueKinetic = getKineticJointTorques(externalForce, tempLeg, EEselection, meanCyclicMotionHipEE);
tempLeg.(EEselection).jointTorqueDynamic = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);
tempLeg.(EEselection).jointTorque = tempLeg.(EEselection).jointTorqueKinetic + tempLeg.(EEselection).jointTorqueDynamic;

% Smoothen result using moving average
for j = 1:length(Leg.(EEselection).jointTorque(1,:))
    tempLeg.(EEselection).jointTorque(:,j) = smooth(tempLeg.(EEselection).jointTorque(:,j));
end

%% Get active and passive torques for spring modeled in parallel with joint
if springInParallelWithJoints
    q0SpringJoint.(EEselection) = mean(tempLeg.(EEselection).q(:,1:end-1)); % Set undeformed spring position to mean position. This can be updated in optimizer.
    [tempLeg.activeTorqueOpt, tempLeg.passiveTorqueOpt] = getActiveAndPassiveTorque(kSpringJointOpt, q0SpringJoint, tempLeg, EEselection, linkCount);
    tempLeg.activePowerOpt  = tempLeg.activeTorqueOpt  .* tempLeg.(EEselection).qdot(:,1:end-1);
    tempLeg.passivePowerOpt = tempLeg.passiveTorqueOpt .* tempLeg.(EEselection).qdot(:,1:end-1);
else
    tempLeg.activeTorqueOpt = tempLeg.(EEselection).jointTorque;
    tempLeg.passiveTorqueOpt = zeros(size(tempLeg.activeTorqueOpt));
end  

%% Get joint power for optimal design
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
if ~springInParallelWithJoints
    tempLeg.activePowerOpt  = tempLeg.(EEselection).jointPower;
    tempLeg.passivePowerOpt = zeros(size(tempLeg.activePowerOpt));
end

%% Get actuator torque and speed as result of gearing between actuator and joint
% This is the required output torque and speed from the actuator to produce
% the joint torque and speed.
for j = 1:linkCount+1
    % gear ratio = actuator speed / joint speed = joint torque/
    % actuator torque
    transmissionGearRatio = robotProperties.transmissionGearRatioOpt.(jointNames(j,:))(selectFrontHind);
    tempLeg.(EEselection).actuatorq(:,j)      = transmissionGearRatio * tempLeg.(EEselection).q(:,j);
    tempLeg.(EEselection).actuatorqdot(:,j)   = transmissionGearRatio * tempLeg.(EEselection).qdot(:,j);
    tempLeg.(EEselection).actuatorTorque(:,j) = (1/transmissionGearRatio) * tempLeg.activeTorqueOpt(:,j);
end

%% Get link mass for optimal design
linkMassOpt = [robotProperties.hip(selectFrontHind).mass, ...
               robotProperties.thigh(selectFrontHind).mass, ...
               robotProperties.shank(selectFrontHind).mass];
if linkCount > 2
    linkMassOpt(end+1) = robotProperties.foot(selectFrontHind).mass;
end

if linkCount > 3
    linkMassOpt(end+1) = robotProperties.phalanges(selectFrontHind).mass;
end
totalLinkMassOpt = sum(linkMassOpt);

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

 [mechEnergyActiveOpt, mechEnergyPerCycleActiveOpt, ~, ~]  = computeEnergyConsumption(tempLeg.activePowerOpt, tempLeg.(EEselection).electricalPower, dt);

%% Return the results of the optimization
optimizationResults.linkLengthsOpt = linkLengths;
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
optimizationResults.activeTorqueOpt = tempLeg.activeTorqueOpt;
optimizationResults.activePowerOpt = tempLeg.activePowerOpt;
optimizationResults.passiveTorqueOpt = tempLeg.passiveTorqueOpt;
optimizationResults.passivePowerOpt = tempLeg.passivePowerOpt;
optimizationResults.kSpringJointOpt = kSpringJointOpt.(EEselection);
optimizationResults.mechEnergyActiveOpt = mechEnergyActiveOpt;
optimizationResults.mechEnergyPerCycleActiveOpt = mechEnergyPerCycleActiveOpt;