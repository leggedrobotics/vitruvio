function penalty = computePenalty(legDesignParameters, actuateJointsDirectly, linkCount, optimizationProperties, quadruped, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex)
% the optimization returns new leg design parameters with unit cm. Convert
% lengths back to m to run the simulation and obtain results with base
% units.
linkLengths = legDesignParameters(1:linkCount+1)/100;
hipAttachmentOffset = legDesignParameters(linkCount+2)/100;
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

%% Inverse kinematics to calculate joint angles for each leg joint as well as xyz coordinates of joints
% Compute qAFE and qDFE based on heuristics when applicable.
if (linkCount > 2)
    if linkCount == 3
        tempLeg.(EEselection).q(:,4) = computeqFinalJoint(Leg, EEselection, configSelection);
    elseif linkCount == 4
        tempLeg.(EEselection).q(:,5) = computeqFinalJoint(Leg, EEselection, configSelection);
    end
end
% inverse kinematics
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);

%% Build robot model with joint angles from inverse kinematics tempLeg
numberOfLoopRepetitions = 1;
viewVisualization = 0;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% The joint accelerations are then computed using finite difference
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);

%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% Energy recuperation 
% no recuperation, set negative power terms to zero
jointPowerInitial = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
for j = 1:length(jointPower)
    for k = 1:length(jointPower(1,:))
        if jointPowerInitial(j,k) < 0
              jointPowerInitial(j,k) = 0;
        end
        if jointPower(j,k) < 0
              jointPower(j,k) = 0;
        end
    end
end

%% Load in penalty weights
W_totalSwingTorque   = optimizationProperties.penaltyWeight.totalSwingTorque;
W_totalStanceTorque   = optimizationProperties.penaltyWeight.totalStanceTorque;
W_totalTorque        = optimizationProperties.penaltyWeight.totalTorque;
W_totalTorqueHFE  = optimizationProperties.penaltyWeight.totalTorqueHFE;
W_swingTorqueHFE  = optimizationProperties.penaltyWeight.swingTorqueHFE;
W_totalqdot     = optimizationProperties.penaltyWeight.totalqdot;
W_totalPower    = optimizationProperties.penaltyWeight.totalPower;
W_maxTorque     = optimizationProperties.penaltyWeight.maxTorque;
W_maxqdot       = optimizationProperties.penaltyWeight.maxqdot;
W_maxPower      = optimizationProperties.penaltyWeight.maxPower;
allowableExtension = optimizationProperties.allowableExtension; % as ratio of total possible extension

%% Compute penalty terms
% For initial and new leg design. The penalty is then computed by
% normalizing the penalty of the new design by the inital design.
torqueInitial.swing  = Leg.(EEselection).jointTorque(1:meanTouchdownIndex.(EEselection), :);
torque.swing  = tempLeg.(EEselection).jointTorque(1:meanTouchdownIndex.(EEselection), :);

torqueInitial.stance = Leg.(EEselection).jointTorque(meanTouchdownIndex.(EEselection)+1:end, :);
torque.stance = tempLeg.(EEselection).jointTorque(meanTouchdownIndex.(EEselection)+1:end, :);

totalSwingTorqueInitial = sum(sum((torqueInitial.swing).^2)); 
totalSwingTorque = sum(sum((torque.swing).^2)); 

totalStanceTorqueInitial = sum(sum((torqueInitial.stance).^2));
totalStanceTorque = sum(sum((torque.stance).^2));

totalTorqueInitial    = sum(sum((Leg.(EEselection).jointTorque).^2)); 
totalTorque    = sum(sum((tempLeg.(EEselection).jointTorque).^2)); 

totalTorqueHFEInitial = sum((Leg.(EEselection).jointTorque(:,2)).^2); 
totalTorqueHFE = sum((tempLeg.(EEselection).jointTorque(:,2)).^2); 

swingTorqueHFEInitial = sum((torqueInitial.swing(:,2)).^2);
swingTorqueHFE = sum((torque.swing(:,2)).^2);

totalqdotInitial     = sum(sum((Leg.(EEselection).qdot).^2));
totalqdot      = sum(sum((tempLeg.(EEselection).qdot).^2));

totalPowerInitial    = sum(sum(jointPowerInitial));
totalPower     = sum(sum(jointPower));

maxTorqueInitial      = max(max(abs(Leg.(EEselection).jointTorque)));
maxTorque      = max(max(abs(tempLeg.(EEselection).jointTorque)));

maxqdotInitial       = max(max(abs(Leg.(EEselection).qdot)));
maxqdot        = max(max(abs(tempLeg.(EEselection).qdot)));

maxPowerInitial      = max(max(jointPowerInitial));
maxPower       = max(max(jointPower));

%% Compute constraint penalty terms
% TRACKING ERROR PENALTY
% impose tracking error penalty if any point has tracking error above an
% allowable threshhold
trackingError = meanCyclicMotionHipEE.(EEselection).position-tempLeg.(EEselection).r.EE;
if max(abs(trackingError)) > 0.01
    trackingErrorPenalty = 100000;
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
    jointBelowEEPenalty = 1000000;
else
    jointBelowEEPenalty = 0;
end

% KFE ABOVE HFE PENALTY - otherwise spider config preferred
% find max z position of KFE and penalize if above origin
maxHeightKFE = max(tempLeg.(EEselection).r.KFE(:,3));
% if non zero, this must be the largest penalty as it is an infeasible solution
if (maxHeightKFE > 0)
    KFEHeightPenalty = 1000000;
else
    KFEHeightPenalty = 0;
end

% OVEREXTENSION PENALTY
if optimizationProperties.penaltyWeight.maximumExtension % if true, calculate and penalize for overzealous extension
    offsetHFE2EEdes = tempLeg.(EEselection).r.HFE - meanCyclicMotionHipEE.(EEselection).position(1:end-2,:); % offset from HFE to desired EE position at all time steps
    maxOffsetHFE2EEdes = max(sqrt(sum(offsetHFE2EEdes.^2,2))); % max euclidian distance from HFE to desired EE position
        if maxOffsetHFE2EEdes > allowableExtension*sum(linkLengths(2:end))
            maximumExtensionPenalty = 100000000;
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
          trackingErrorPenalty + ...
          jointBelowEEPenalty + ...
          maximumExtensionPenalty + ...
          KFEHeightPenalty;
end