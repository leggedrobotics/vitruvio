% the optimization returns new leg design parameters with unit cm. Convert
% lengths back to m to run the simulation and obtain results with base
% units.

function penalty = computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction)
jointNames = ['HAA'; 'HFE'; 'KFE'; 'DFE'; 'AFE'];

% Design parameters always start with [links(3-5), hip offset]
if linkCount>2 && heuristic.torqueAngle.apply
    kTorsionalSpring = legDesignParameters(end-1); % Spring parameters are at end of leg design parameters vector
    thetaLiftoff_des = legDesignParameters(end);
else 
    kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
    thetaLiftoff_des = heuristic.torqueAngle.thetaLiftoff_des;
end

linkLengths = legDesignParameters(1:linkCount+1);
hipAttachmentOffset = legDesignParameters(linkCount+2);
transmissionGearRatio = legDesignParameters(linkCount+3:2*linkCount+3);

tempLeg.base = Leg.base;

% Update robot properties with newly computed leg design parameters, unit in meters
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
if linkCount == 4
    robotProperties.phalanges(selectFrontHind).mass = robotProperties.legDensity.phalanges(selectFrontHind) * pi*(robotProperties.phalanges(selectFrontHind).radius)^2 * linkLengths(5);
end

%% qAFE, qDFE torque based heuristic computation
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    % Save the updated spring parameters back into the heuristic struct to
    % be used in the next iteration of the simulation
    heuristic.torqueAngle.kTorsionalSpring = kTorsionalSpring;
    heuristic.torqueAngle.thetaLiftoff_des = thetaLiftoff_des;
    [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, configSelection, hipParalleltoBody);
    EE_force = Leg.(EEselection).force(1,1:3);
    rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
    qPrevious = qLiftoff.(EEselection);
else
    qLiftoff.(EEselection) = 0; % if the heuristic does not apply
end

%% Inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% Build robot model with joint angles from inverse kinematics tempLeg
numberOfLoopRepetitions = 1;
viewVisualization = 0;
saveFiguresToPDF  = 0;
optimized = true;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointDirectly, hipAttachmentOffset, linkCount, robotProperties, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody, dataExtraction, optimized, saveFiguresToPDF);

%% Get joint velocities and accelerations with finite differences
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg, dt);
% Smoothen the velocity and acceleration data using moving average
for j = 1:length(Leg.(EEselection).qdot(1,:))
    tempLeg.(EEselection).qdot(:,j) = smooth(tempLeg.(EEselection).qdot(:,j));
    tempLeg.(EEselection).qdotdot(:,j) = smooth(tempLeg.(EEselection).qdotdot(:,j));
end
%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);
% Smoothen the torque using moving average
for j = 1:length(Leg.(EEselection).jointTorque(1,:))
    tempLeg.(EEselection).jointTorque(:,j) = smooth(tempLeg.(EEselection).jointTorque(:,j));
end

%% Mechanical power at joint
jointPowerInitial = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(:,1:end-1);
tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
jointPower = tempLeg.(EEselection).jointPower;
% Set negative terms to zero
jointPowerInitial(jointPowerInitial<0) = 0;
jointPower(jointPower<0) = 0;

%% Get actuator torque and speed as result of gearing between actuator and joint
% This is the required output torque and speed from the actuator to produce
% the joint torque and speed.
for j = 1:linkCount+1
    % gear ratio = actuator speed / joint speed = joint torque/
    % actuator torque
    tempLeg.(EEselection).actuatorq(:,j)      = transmissionGearRatio(j) * tempLeg.(EEselection).q(:,j);
    tempLeg.(EEselection).actuatorqdot(:,j)   = transmissionGearRatio(j) * tempLeg.(EEselection).qdot(:,j);
    tempLeg.(EEselection).actuatorTorque(:,j) = (1/transmissionGearRatio(j)) * tempLeg.(EEselection).jointTorque(:,j);
end

%% Get electrical power and efficiency at each operating point
[tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

 %% Energy consumption 
% here we assume no recuperation of energy possible. This means the
% negative power terms are set to zero. Integral computed using trapezoids.
[tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(jointPower, tempLeg.(EEselection).electricalPower, dt);
%  tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = tempLeg.metaParameters.mechEnergyPerCycle.(EEselection) + sum(tempLeg.metaParameters.mechEnergyPerCycle.(EEselection));
%  tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = tempLeg.metaParameters.elecEnergyPerCycle.(EEselection) + sum(tempLeg.metaParameters.elecEnergyPerCycle.(EEselection));    

%% Load in penalty weights
W_totalSwingTorque    = optimizationProperties.penaltyWeight.totalSwingTorque;
W_totalStanceTorque   = optimizationProperties.penaltyWeight.totalStanceTorque;
W_totalTorque         = optimizationProperties.penaltyWeight.totalTorque;
W_totalTorqueHFE      = optimizationProperties.penaltyWeight.totalTorqueHFE;
W_swingTorqueHFE      = optimizationProperties.penaltyWeight.swingTorqueHFE;
W_totalqdot           = optimizationProperties.penaltyWeight.totalqdot;
W_totalPower          = optimizationProperties.penaltyWeight.totalPower;
W_totalMechEnergy     = optimizationProperties.penaltyWeight.totalMechEnergy;
W_totalElecEnergy     = optimizationProperties.penaltyWeight.totalElecEnergy;
W_averageEfficiency   = optimizationProperties.penaltyWeight.averageEfficiency;
W_maxTorque           = optimizationProperties.penaltyWeight.maxTorque;
W_maxqdot             = optimizationProperties.penaltyWeight.maxqdot;
W_maxPower            = optimizationProperties.penaltyWeight.maxPower;
W_antagonisticPower   = optimizationProperties.penaltyWeight.antagonisticPower;
allowableExtension    = optimizationProperties.allowableExtension; % as ratio of total possible extension

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
maximumExtensionPenalty = 0;

%% Compute penalty terms          
if W_totalSwingTorque
    if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque
        torqueInitial.swing  = Leg.(EEselection).actuatorTorque(1:meanTouchdownIndex.(EEselection), 1:end-1);
        torque.swing  = tempLeg.(EEselection).actuatorTorque(1:meanTouchdownIndex.(EEselection), 1:end-1);
    else
        torqueInitial.swing  = Leg.(EEselection).actuatorTorque(1:meanTouchdownIndex.(EEselection), :);
        torque.swing  = tempLeg.(EEselection).actuatorTorque(1:meanTouchdownIndex.(EEselection), :);
    end
    totalSwingTorqueInitial = sum(sum((torqueInitial.swing).^2));
    totalSwingTorque = sum(sum((torque.swing).^2)); 
end

if W_totalStanceTorque
    if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque    
        torqueInitial.stance = Leg.(EEselection).actuatorTorque(meanTouchdownIndex.(EEselection)+1:end, 1:end-1);
        torque.stance = tempLeg.(EEselection).actuatorTorque(meanTouchdownIndex.(EEselection)+1:end, 1:end-1);

    else
        torqueInitial.stance = Leg.(EEselection).actuatorTorque(meanTouchdownIndex.(EEselection)+1:end, :);
        torque.stance = tempLeg.(EEselection).actuatorTorque(meanTouchdownIndex.(EEselection)+1:end, :);
    end
    totalStanceTorqueInitial = sum(sum(abs(torqueInitial.stance)));
    totalStanceTorque = sum(sum(abs(torque.stance)));
end

if W_totalTorque
    if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque
        totalTorqueInitial = sum(sum(abs(Leg.(EEselection).actuatorTorque(:,1:end-1)))); 
        totalTorque = sum(sum(abs(tempLeg.(EEselection).actuatorTorque(:,1:end-1))));
    else
        totalTorqueInitial = sum(sum(abs(Leg.(EEselection).actuatorTorque))); 
        totalTorque = sum(sum(abs(tempLeg.(EEselection).actuatorTorque))); 
    end
end

if W_totalTorqueHFE
    totalTorqueHFEInitial = sum(abs(Leg.(EEselection).actuatorTorque(:,2))); 
    totalTorqueHFE = sum(abs(tempLeg.(EEselection).actuatorTorque(:,2))); 
end

if W_swingTorqueHFE
    swingTorqueHFEInitial = sum(abs(torqueInitial.swing(:,2)));
    swingTorqueHFE = sum(abs(torque.swing(:,2)));
end

if W_totalqdot
    totalqdotInitial     = sum(sum(abs(Leg.(EEselection).actuatorqdot)));
    totalqdot      = sum(sum(abs(tempLeg.(EEselection).actuatorqdot)));
end

if W_totalPower
    totalPowerInitial    = sum(sum(jointPowerInitial));
    totalPower     = sum(sum(jointPower));
end

if W_totalMechEnergy
    if linkCount>2 && heuristic.torqueAngle.apply   
        totalMechEnergyInitial = sum(Leg.(EEselection).mechEnergy(end,1:end-1)); % sum of mech energy consumed over all active joints during the motion
        totalMechEnergy    = sum(tempLeg.(EEselection).mechEnergy(end,1:end-1));
    else
        totalMechEnergyInitial = sum(Leg.(EEselection).mechEnergy(end,:));
        totalMechEnergy    = sum(tempLeg.(EEselection).mechEnergy(end,:));
    end
end

if W_totalElecEnergy
    if linkCount>2 && heuristic.torqueAngle.apply     
        totalElecEnergyInitial    = sum(Leg.(EEselection).elecEnergy(end,1:end-1)); % sum of elec energy consumed over all active joints during the motion
        totalElecEnergy    = sum(tempLeg.(EEselection).elecEnergy(end,1:end-1));
    else
        totalElecEnergyInitial    = sum(Leg.(EEselection).elecEnergy(end,:)); % sum of elec energy consumed over all joints during the motion
        totalElecEnergy    = sum(tempLeg.(EEselection).elecEnergy(end,:));        
    end
    
end

if W_averageEfficiency    
    averageEfficiencyInitial = mean(mean(Leg.(EEselection).operatingPointEfficiency));
    averageEfficiency = mean(mean(tempLeg.(EEselection).operatingPointEfficiency));
end

if W_maxTorque
    if linkCount>2 && heuristic.torqueAngle.apply         
        maxTorqueInitial = max(max(abs(Leg.(EEselection).actuatorTorque(:,1:end-1))));
        maxTorque = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,1:end-1))));
    else
        maxTorqueInitial = max(max(abs(Leg.(EEselection).actuatorTorque)));
        maxTorque = max(max(abs(tempLeg.(EEselection).actuatorTorque)));        
    end
end

if W_maxqdot
    if linkCount>2 && heuristic.torqueAngle.apply             
        maxqdotInitial = max(max(abs(Leg.(EEselection).actuatorqdot(:,1:end-1))));
        maxqdot = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,1:end-1))));
    else
    	maxqdotInitial = max(max(abs(Leg.(EEselection).actuatorqdot)));
        maxqdot = max(max(abs(tempLeg.(EEselection).actuatorqdot)));
    end
end

if W_maxPower
    if linkCount>2 && heuristic.torqueAngle.apply                 
        maxPowerInitial = max(max(jointPowerInitial(:,1:end-1)));
        maxPower = max(max(jointPower(:,1:end-1)));
    else
        maxPowerInitial = max(max(jointPowerInitial));
        maxPower = max(max(jointPower));        
    end
end

if W_antagonisticPower
    antagonisticPowerInitial      = 0.5*(sum(sum(jointPowerInitial)) - sum(sum(abs(jointPowerInitial))));
    antagonisticPower             = 0.5*(sum(sum(jointPower)) - sum(sum(abs(jointPower))));
end

%% Soft constraints due to actuator limits
% get the maximum joint torque speed and power for each joint
% joint torque speed and power limits for 2 link leg
maxTorqueHAA = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,1))));
maxTorqueHFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,2))));
maxTorqueKFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,3))));
maxTorqueAFE = 0;
maxTorqueDFE = 0;

maxqdotHAA   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,1))));
maxqdotHFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,2))));
maxqdotKFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,3))));
maxqdotAFE   = 0;
maxqdotDFE   = 0;

maxPowerHAA  = max(max(jointPower(:,1)));
maxPowerHFE  = max(max(jointPower(:,2)));
maxPowerKFE  = max(max(jointPower(:,3)));
maxPowerAFE  = 0;
maxPowerDFE  = 0;

% overwrite the AFE and DFE values for 3 and 4 link leg
if linkCount == 3
    maxTorqueAFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,4))));
    maxqdotAFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,4))));
    maxPowerAFE  = max(max(jointPower(:,4)));
end
if linkCount == 4
    maxTorqueAFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,4))));
    maxTorqueDFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,5))));
    maxqdotAFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,4))));
    maxqdotDFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,5))));
    maxPowerAFE  = max(max(jointPower(:,4)));
    maxPowerDFE  = max(max(jointPower(:,5)));
end

maxActuatorTorque = [maxTorqueHAA, maxTorqueHFE, maxTorqueKFE, maxTorqueAFE, maxTorqueDFE];
maxActuatorqdot   = [maxqdotHAA,   maxqdotHFE,   maxqdotKFE,   maxqdotAFE,   maxqdotDFE];
maxJointPower  = [maxPowerHAA,  maxPowerHFE,  maxPowerKFE,  maxPowerAFE,  maxPowerDFE];

%% Compute max joint torque, speed and power limit violation penalties
% initialize penalties for exceeding actuator limits
torqueLimitPenalty = 0; speedLimitPenalty = 0; powerLimitPenalty = 0;

if imposeJointLimits.maxTorque && any(maxActuatorTorque > maxTorqueLimit)
    torqueLimitPenalty = 5;
end

if imposeJointLimits.maxqdot && any(maxActuatorqdot > maxqdotLimit)
    speedLimitPenalty = 5;
end

if imposeJointLimits.maxPower && any(maxJointPower > maxPowerLimit)
    powerLimitPenalty = 5;
end

%% Compute constraint penalty terms
% TRACKING ERROR PENALTY
% impose tracking error penalty if any point has tracking error above an
% allowable threshhold
trackingError = meanCyclicMotionHipEE.(EEselection).position(1:length(tempLeg.(EEselection).r.EE),:)-tempLeg.(EEselection).r.EE;
trackingError = sqrt(sum(trackingError.^2,2)); % Magnitude of tracking error at each timestep
if max(trackingError) > 0.001 % If tracking error ever exceeds this tolerance, impose a penalty.
    trackingErrorPenalty = 10;
else
    trackingErrorPenalty = 0;
end

%% JOINT TOO CLOSE TO GROUND
% Requires us to bring in hipNomZ
% For flat ground, ground height is the distance CoM z position.
    groundHeight = -Leg.base.position.(EEselection)(:,3);
    % Get the height of the lowest joint at each time step.
    for i = 1:length(tempLeg.(EEselection).r.HAA(:,3))
        jointHeight = []; % Initialize jointHeight to be empty
        for j = 1:linkCount+1 
            % Fill in joint heights for all of the joints in the leg.
            jointHeight = [jointHeight, tempLeg.(EEselection).r.(jointNames(j,:))(i,3)];         
        end
        lowestJoint(i,1) =  min(jointHeight);
    end              
    % Apply penalty if the lowest joint is ever too close to the ground                 
    if any(lowestJoint - groundHeight(1:length(lowestJoint)) < 0.1)
        jointBelowGroundPenalty = 10;
    else
        jointBelowGroundPenalty = 0;
    end

%% KFE ABOVE HFE PENALTY - otherwise spider config preferred
% find max z position of KFE and penalize if above origin
    for i = 1:length(tempLeg.(EEselection).r.HAA(:,3))
        jointHeightHFE(i,1) = tempLeg.(EEselection).r.HFE(i,3);
        jointHeightKFE(i,1) = tempLeg.(EEselection).r.KFE(i,3);
    end   

    % Apply penalty if KFE joint is ever above HFE joint
    if any(jointHeightKFE > jointHeightHFE)
        KFEAboveHFEPenalty = 10;
    else
        KFEAboveHFEPenalty = 0;
    end
    
%% Last joint in front of end effector (Control spring deform. direction)
    % Applies only if using spring heuristic
    if linkCount>2 && heuristic.torqueAngle.apply                 
        for i = 1:length(tempLeg.(EEselection).r.HAA(:,3))
            if linkCount == 3
                jointPositionxLastJoint(i,1) = tempLeg.(EEselection).r.AFE(i,1);
            else
                jointPositionxLastJoint(i,1) = tempLeg.(EEselection).r.DFE(i,1);
            end            
        end   
        % Apply penalty if last joint (AFE or DFE) is ever in front of EE
        if any(jointPositionxLastJoint > tempLeg.(EEselection).r.EE(i,1))
            lastJointInFrontOfEEPenalty = 10;
        else
            lastJointInFrontOfEEPenalty = 0;
        end 
    else
        lastJointInFrontOfEEPenalty = 0;
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
penalty = W_totalTorque         * (totalTorque/totalTorqueInitial) + ...
          W_totalTorqueHFE      * (totalTorqueHFE/totalTorqueHFEInitial) + ...
          W_swingTorqueHFE      * (swingTorqueHFE/swingTorqueHFEInitial) + ...
          W_totalSwingTorque    * (totalSwingTorque/totalSwingTorqueInitial) + ...
          W_totalStanceTorque   * (totalStanceTorque/totalStanceTorqueInitial) + ...
          W_totalqdot           * (totalqdot/totalqdotInitial)     + ...
          W_totalPower          * (totalPower/totalPowerInitial)   + ...
          W_maxTorque           * (maxTorque/maxTorqueInitial)     + ...
          W_maxqdot             * (maxqdot/maxqdotInitial)         + ...
          W_maxPower            * (maxPower/maxPowerInitial)     + ...
          W_antagonisticPower   * (antagonisticPower/antagonisticPowerInitial)     + ... 
          W_totalMechEnergy     * (totalMechEnergy/totalMechEnergyInitial)     + ... 
          W_totalElecEnergy     * (totalElecEnergy/totalElecEnergyInitial)     + ...
          W_averageEfficiency   / (averageEfficiency/averageEfficiencyInitial)     + ... % minimize 1/efficiency
          trackingErrorPenalty        + ...
          jointBelowGroundPenalty     + ...
          KFEAboveHFEPenalty          + ...
          lastJointInFrontOfEEPenalty + ... % only applies for spring at final joint to control the spring deformation direction
          optimizationProperties.penaltyWeight.maximumExtension * maximumExtensionPenalty + ...
          torqueLimitPenalty + ...
          speedLimitPenalty  + ...
          powerLimitPenalty;
end