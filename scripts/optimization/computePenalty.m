% the optimization returns new leg design parameters with unit cm. Convert
% lengths back to m to run the simulation and obtain results with base
% units.

function penalty = computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointsDirectly, linkCount, optimizationProperties, quadruped, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex, actuatorEfficiency, actuatorSelection)
jointNames = ['HAA'; 'HFE'; 'KFE'; 'DFE'; 'AFE'];

kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
thetaLiftoff_des = heuristic.torqueAngle.thetaLiftoff_des;
linkLengths = legDesignParameters(1:linkCount+1)/100;
hipAttachmentOffset = legDesignParameters(linkCount+2)/100;
tempLeg.base = Leg.base;
% Update quadruped properties with newly computed leg design parameters, unit in meters
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
if linkCount == 4
    quadruped.phalanges(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.phalanges(selectFrontHind).radius)^2 * linkLengths(5);
end

%% qAFE, qDFE torque based heuristic computation
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    [qLiftoff.(EEselection{1})] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
    EE_force = Leg.(EEselection{1}).force(1,1:3);
    rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection{1})(1,2); % rotation of body about inertial y
    qPrevious = qLiftoff.(EEselection{1});
    [springTorque.(EEselection{1}), springDeformation.(EEselection{1})] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);      
else
    qLiftoff.(EEselection) = 0; % if the heuristic does not apply
end

%% inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% Build robot model with joint angles from inverse kinematics tempLeg
numberOfLoopRepetitions = 1;
viewVisualization = 0;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);

%% Get joint velocities and accelerations with finite differences
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, Leg);

%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% Mechanical power
jointPowerInitial                = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(:,1:end-1);
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
jointPower = tempLeg.(EEselection).jointPower;

%% Get electrical power and efficiency at each operating point
[tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

 %% Energy consumption 
% here we assume no recuperation of energy possible. This means the
% negative power terms are set to zero.
[tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(tempLeg, EEselection, dt);
 tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = tempLeg.metaParameters.mechEnergyPerCycle.(EEselection) + sum(tempLeg.metaParameters.mechEnergyPerCycle.(EEselection));
 tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = tempLeg.metaParameters.elecEnergyPerCycle.(EEselection) + sum(tempLeg.metaParameters.elecEnergyPerCycle.(EEselection));    

%% Load in penalty weights
W_totalSwingTorque   = optimizationProperties.penaltyWeight.totalSwingTorque;
W_totalStanceTorque   = optimizationProperties.penaltyWeight.totalStanceTorque;
W_totalTorque        = optimizationProperties.penaltyWeight.totalTorque;
W_totalTorqueHFE  = optimizationProperties.penaltyWeight.totalTorqueHFE;
W_swingTorqueHFE  = optimizationProperties.penaltyWeight.swingTorqueHFE;
W_totalqdot     = optimizationProperties.penaltyWeight.totalqdot;
W_totalPower    = optimizationProperties.penaltyWeight.totalPower;
W_totalMechEnergy   = optimizationProperties.penaltyWeight.totalMechEnergy;
W_totalElecEnergy   = optimizationProperties.penaltyWeight.totalElecEnergy;
W_averageEfficiency  =  optimizationProperties.penaltyWeight.averageEfficiency;
W_maxTorque     = optimizationProperties.penaltyWeight.maxTorque;
W_maxqdot       = optimizationProperties.penaltyWeight.maxqdot;
W_maxPower      = optimizationProperties.penaltyWeight.maxPower;
W_antagonisticPower  = optimizationProperties.penaltyWeight.antagonisticPower;
allowableExtension = optimizationProperties.allowableExtension; % as ratio of total possible extension

% initialize torque, qdot and power limits.
maxTorqueLimit = [0 0 0 0 0];    
maxqdotLimit   = [0 0 0 0 0];    
maxPowerLimit  = [0 0 0 0 0]; 

for i = 1:linkCount+1
    jointSelection = jointNames(i,:);
    if imposeJointLimits.maxTorque
        maxTorqueLimit(1,i) = optimizationProperties.bounds.maxTorqueLimit.(jointSelection);
    end

    if imposeJointLimits.maxqdot
        maxqdotLimit(1,i) = optimizationProperties.bounds.maxqdotLimit.(jointSelection);
    end

    if imposeJointLimits.maxPower
        maxPowerLimit(1,i) = optimizationProperties.bounds.maxPowerLimit.(jointSelection);
    end
end

%% Initialize penalty terms
totalTorque = 0;
totalTorqueInitial = 1;
totalTorqueHFE = 0;
totalTorqueHFEInitial = 1;
swingTorqueHFE = 0;
swingTorqueHFEInitial = 1;
totalSwingTorque = 0;
totalSwingTorqueInitial = 1;
totalStanceTorque = 0;
totalStanceTorqueInitial = 1;
totalqdot = 0;
totalqdotInitial = 1;
totalPower = 0;
totalPowerInitial = 1;
maxTorque = 0;
maxTorqueInitial = 1;
maxqdot = 0;
maxqdotInitial = 1;
maxPower = 0;
maxPowerInitial = 1;
totalMechEnergy = 0;
totalMechEnergyInitial = 1;
totalElecEnergy = 0;
totalElecEnergyInitial = 1;
averageEfficiency = 1;
averageEfficiencyInitial = 1;
antagonisticPower = 1;
antagonisticPowerInitial = 1;

%% Compute penalty terms          
if W_totalSwingTorque
    torqueInitial.swing  = Leg.(EEselection).jointTorque(1:meanTouchdownIndex.(EEselection), :);
    torque.swing  = tempLeg.(EEselection).jointTorque(1:meanTouchdownIndex.(EEselection), :);
    totalSwingTorqueInitial = sum(sum((torqueInitial.swing).^2));
    totalSwingTorque = sum(sum((torque.swing).^2)); 
end
if W_totalStanceTorque
    torqueInitial.stance = Leg.(EEselection).jointTorque(meanTouchdownIndex.(EEselection)+1:end, :);
    torque.stance = tempLeg.(EEselection).jointTorque(meanTouchdownIndex.(EEselection)+1:end, :);
    totalStanceTorqueInitial = sum(sum((torqueInitial.stance).^2));
    totalStanceTorque = sum(sum((torque.stance).^2));
end
if W_totalTorque
    totalTorqueInitial    = sum(sum((Leg.(EEselection).jointTorque).^2)); 
    totalTorque    = sum(sum((tempLeg.(EEselection).jointTorque).^2)); 
end
if W_totalTorqueHFE
    totalTorqueHFEInitial = sum((Leg.(EEselection).jointTorque(:,2)).^2); 
    totalTorqueHFE = sum((tempLeg.(EEselection).jointTorque(:,2)).^2); 
end
if W_swingTorqueHFE
    swingTorqueHFEInitial = sum((torqueInitial.swing(:,2)).^2);
    swingTorqueHFE = sum((torque.swing(:,2)).^2);
end
if W_totalqdot
    totalqdotInitial     = sum(sum((Leg.(EEselection).qdot).^2));
    totalqdot      = sum(sum((tempLeg.(EEselection).qdot).^2));
end
if W_totalPower
    totalPowerInitial    = sum(sum(jointPowerInitial));
    totalPower     = sum(sum(jointPower));
end
if W_totalMechEnergy
    totalMechEnergyInitial    = sum(sum(Leg.(EEselection).mechEnergy));
    totalMechEnergy    = sum(sum(tempLeg.(EEselection).mechEnergy));
end
if W_totalElecEnergy
    totalElecEnergyInitial    = sum(sum(Leg.(EEselection).elecEnergy));
    totalElecEnergy    = sum(sum(tempLeg.(EEselection).elecEnergy));
end
if W_averageEfficiency
    % Efficiency is NaN in quadrants I and III. Set this to 0.0001 to penalize it avoid
    % having NaN values 
    Leg.(EEselection).operatingPointEfficiency(isnan(Leg.(EEselection).operatingPointEfficiency)) = 0.0001;
    tempLeg.(EEselection).operatingPointEfficiency(isnan(tempLeg.(EEselection).operatingPointEfficiency)) = 0.0001;    
    
    averageEfficiencyInitial    = mean(mean(Leg.(EEselection).operatingPointEfficiency));
    averageEfficiency    = mean(mean(tempLeg.(EEselection).operatingPointEfficiency));
end
if W_maxTorque
    maxTorqueInitial      = max(max(abs(Leg.(EEselection).jointTorque)));
    maxTorque      = max(max(abs(tempLeg.(EEselection).jointTorque)));
end
if W_maxqdot
    maxqdotInitial       = max(max(abs(Leg.(EEselection).qdot)));
    maxqdot        = max(max(abs(tempLeg.(EEselection).qdot)));
end
if W_maxPower
    maxPowerInitial      = max(max(jointPowerInitial));
    maxPower       = max(max(jointPower));
end
if W_antagonisticPower
    antagonisticPowerInitial      = 0.5*(sum(sum(jointPowerInitial)) - sum(sum(abs(jointPowerInitial))));
    antagonisticPower             = 0.5*(sum(sum(jointPower)) - sum(sum(abs(jointPower))));
end

%% get the maximum joint torque speed and power for each joint
% joint torque speed and power limits for 2 link leg
maxTorqueHAA = max(max(abs(tempLeg.(EEselection).jointTorque(:,1))));
maxTorqueHFE = max(max(abs(tempLeg.(EEselection).jointTorque(:,2))));
maxTorqueKFE = max(max(abs(tempLeg.(EEselection).jointTorque(:,3))));
maxTorqueAFE = 0;
maxTorqueDFE = 0;

maxqdotHAA   = max(max(abs(tempLeg.(EEselection).qdot(:,1))));
maxqdotHFE   = max(max(abs(tempLeg.(EEselection).qdot(:,2))));
maxqdotKFE   = max(max(abs(tempLeg.(EEselection).qdot(:,3))));
maxqdotAFE   = 0;
maxqdotDFE   = 0;

maxPowerHAA  = max(max(jointPower(:,1)));
maxPowerHFE  = max(max(jointPower(:,2)));
maxPowerKFE  = max(max(jointPower(:,3)));
maxPowerAFE  = 0;
maxPowerDFE  = 0;

% overwrite the AFE and DFE values for 3 and 4 link leg
if linkCount == 3
    maxTorqueAFE = max(max(abs(tempLeg.(EEselection).jointTorque(:,4))));
    maxqdotAFE   = max(max(abs(tempLeg.(EEselection).qdot(:,4))));
    maxPowerAFE  = max(max(jointPower(:,4)));
end
if linkCount == 4
    maxTorqueAFE = max(max(abs(tempLeg.(EEselection).jointTorque(:,4))));
    maxTorqueDFE = max(max(abs(tempLeg.(EEselection).jointTorque(:,5))));
    maxqdotAFE   = max(max(abs(tempLeg.(EEselection).qdot(:,4))));
    maxqdotDFE   = max(max(abs(tempLeg.(EEselection).qdot(:,5))));
    maxPowerAFE  = max(max(jointPower(:,4)));
    maxPowerDFE  = max(max(jointPower(:,5)));
end

maxJointTorque = [maxTorqueHAA, maxTorqueHFE, maxTorqueKFE, maxTorqueAFE, maxTorqueDFE];
maxJointqdot   = [maxqdotHAA,   maxqdotHFE,   maxqdotKFE,   maxqdotAFE,   maxqdotDFE];
maxJointPower  = [maxPowerHAA,  maxPowerHFE,  maxPowerKFE,  maxPowerAFE,  maxPowerDFE];

%% Compute max joint torque, speed and power limit violation penalties
% initialize penalties for exceeding actuator limits
torqueLimitPenalty = 0; speedLimitPenalty = 0; powerLimitPenalty = 0;

if (imposeJointLimits.maxTorque) && (any(maxJointTorque > maxTorqueLimit))
    torqueLimitPenalty = 5;
end

if (imposeJointLimits.maxqdot) && (any(maxJointqdot > maxqdotLimit))
    speedLimitPenalty = 5;
end

if (imposeJointLimits.maxPower) && (any(maxJointPower > maxPowerLimit))
    powerLimitPenalty = 5;
end

%% Compute constraint penalty terms
% TRACKING ERROR PENALTY
% impose tracking error penalty if any point has tracking error above an
% allowable threshhold
trackingError = meanCyclicMotionHipEE.(EEselection).position(1:end-2,:)-tempLeg.(EEselection).r.EE;
if max(abs(trackingError)) > 0.001
    trackingErrorPenalty = 10;
else
    trackingErrorPenalty = 0;
end

% JOINT BELOW GROUND PENALTY
% find lowest joint and penalize if it is below the EE's lowest point ie
% penetrating the ground
lowestJoint =  min([min(tempLeg.(EEselection).r.HAA(:,3)), ...
                    min(tempLeg.(EEselection).r.HFE(:,3)), ...
                    min(tempLeg.(EEselection).r.KFE(:,3)), ...
                    min(tempLeg.(EEselection).r.AFE(:,3)),  ...
                    min(tempLeg.(EEselection).r.DFE(:,3))]);
                
% if non zero, this must be the largest penalty as it is an infeasible solution
if (lowestJoint < min(min(tempLeg.(EEselection).r.EE(:,3))))
    jointBelowEEPenalty = 10;
else
    jointBelowEEPenalty = 0;
end

% KFE ABOVE HFE PENALTY - otherwise spider config preferred
% find max z position of KFE and penalize if above origin
maxHeightKFE = max(tempLeg.(EEselection).r.KFE(:,3));
% if non zero, this must be the largest penalty as it is an infeasible solution
if (maxHeightKFE > 0)
    KFEHeightPenalty = 10;
else
    KFEHeightPenalty = 0;
end

% KFE WITHIN x TOL / IN FRONT OF HFE - for hopper
% find z position of ground (initial EE height)
maxPosKFE_x = max(tempLeg.(EEselection).r.KFE(:,1));
if (abs(maxPosKFE_x) > 0.01)
    KFEPosPenalty_x = 5;
else
    KFEPosPenalty_x = 0;
end

% OVEREXTENSION PENALTY
if optimizationProperties.penaltyWeight.maximumExtension % if true, calculate and penalize for overzealous extension
    offsetHFE2EEdes = tempLeg.(EEselection).r.HFE - meanCyclicMotionHipEE.(EEselection).position(1:end-2,:); % offset from HFE to desired EE position at all time steps
    maxOffsetHFE2EEdes = max(sqrt(sum(offsetHFE2EEdes.^2,2))); % max euclidian distance from HFE to desired EE position
        if maxOffsetHFE2EEdes > allowableExtension*sum(linkLengths(2:end))
            maximumExtensionPenalty = 10;
        else 
            maximumExtensionPenalty = 0;
    end
end

%% Compute penalty
penalty = W_totalTorque * (totalTorque/totalTorqueInitial) + ...
          W_totalTorqueHFE * (totalTorqueHFE/totalTorqueHFEInitial) + ...
          W_swingTorqueHFE * (swingTorqueHFE/swingTorqueHFEInitial) + ...
          W_totalSwingTorque * (totalSwingTorque/totalSwingTorqueInitial) + ...
          W_totalStanceTorque * (totalStanceTorque/totalStanceTorqueInitial) + ...
          W_totalqdot * (totalqdot/totalqdotInitial)     + ...
          W_totalPower * (totalPower/totalPowerInitial)   + ...
          W_maxTorque * (maxTorque/maxTorqueInitial)     + ...
          W_maxqdot * (maxqdot/maxqdotInitial)         + ...
          W_maxPower * (maxPower/maxPowerInitial)     + ...
          W_antagonisticPower * (antagonisticPower/antagonisticPowerInitial)     + ... 
          W_totalMechEnergy * (totalMechEnergy/totalMechEnergyInitial)     + ... 
          W_totalElecEnergy * (totalElecEnergy/totalElecEnergyInitial)     + ...
          W_averageEfficiency / (averageEfficiency/averageEfficiencyInitial)     + ... % minimize 1/efficiency
          trackingErrorPenalty + ...
          jointBelowEEPenalty + ...
          maximumExtensionPenalty + ...
          KFEHeightPenalty + ...
          KFEPosPenalty_x + ...
          torqueLimitPenalty + ...
          speedLimitPenalty + ...
          powerLimitPenalty;
end