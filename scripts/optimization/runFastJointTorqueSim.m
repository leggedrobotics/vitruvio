function penalty = runFastJointTorqueSim(l_hipAttachmentOffset, linkCount, optimizationProperties, quadruped, linkLengths, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody)
%% Get quadruped properties 
% Update link lengths, unit in meters
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;
if (linkCount == 3) || (linkCount == 4)
    quadruped.foot(selectFrontHind).length = linkLengths(4)/100;
end
if linkCount == 4
    quadruped.phalanges(selectFrontHind).length = linkLengths(5)/100;
end

% Update link mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1)/100;
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2)/100;
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3)/100;
if (linkCount == 3) || (linkCount == 4)
    quadruped.foot(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.foot(selectFrontHind).radius)^2 * linkLengths(4)/100;
end
if linkCount == 4
    quadruped.phalanges(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.phalanges(selectFrontHind).radius)^2 * linkLengths(5)/100;
end
%% Inverse kinematics to calculate joint angles for each leg joint as well as xyz coordinates of joints
[tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(l_hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody);

%% Build robot model with joint angles from inverse kinematics tempLeg
numberOfLoopRepetitions = 1;
viewVisualization = 0;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(l_hipAttachmentOffset, linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% The joint accelerations are then computed using finite difference
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);

%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);

%% Regenerative breaking
% no regenerative breaking, set negative power terms to zero
jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(1:end-2,1:end-1);
for j = 1:length(jointPower)
    for k = 1:length(jointPower(1,:))
        if jointPower(j,k) < 0
              jointPower(j,k) = 0;
        end
    end
end

%% Load in penalty weights
W_totalTorque   = optimizationProperties.penaltyWeight.totalTorque;
W_totalqdot     = optimizationProperties.penaltyWeight.totalqdot;
W_totalPower    = optimizationProperties.penaltyWeight.totalPower;
W_maxTorque     = optimizationProperties.penaltyWeight.maxTorque;
W_maxqdot       = optimizationProperties.penaltyWeight.maxqdot;
W_maxPower      = optimizationProperties.penaltyWeight.maxPower;
W_trackingError = optimizationProperties.penaltyWeight.trackingError;

allowableExtension   = optimizationProperties.allowableExtension;

%% Compute penalty terms
totalTorque   = sum(sum((tempLeg.(EEselection).jointTorque).^2)); 
totalqdot     = sum(sum((tempLeg.(EEselection).qdot).^2));
totalPower    = sum(sum(jointPower));
maxTorque     = max(max(abs(tempLeg.(EEselection).jointTorque)));
maxqdot       = max(max(abs(tempLeg.(EEselection).qdot)));
maxPower      = max(max(abs((tempLeg.(EEselection).jointTorque).*(tempLeg.(EEselection).qdot(1:end-2,1:end-1)))));

%% tracking error penalty
% impose tracking error penalty if any point has tracking error above an
% allowable threshhold
trackingError = meanCyclicMotionHipEE.(EEselection).position-tempLeg.(EEselection).r.EE;
if max(abs(trackingError)) > 0.01
    trackingErrorPenalty = 100000;
    else trackingErrorPenalty = 0;
end

%% joint below ground penalty
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

%% maximum allowable extension exceeded penalty
if optimizationProperties.penaltyWeight.trackingError
    offsetHFE2EEdes = tempLeg.(EEselection).r.HFE - meanCyclicMotionHipEE.(EEselection).position; % offset from HFE to desired EE position
    maxOffsetHFE2EEdes = max(sqrt(sum(offsetHFE2EEdes.^2,2))); % max euclidian distance from HFE to desired EE position
        if maxOffsetHFE2EEdes > allowableExtension*sum(linkLengths(2:end)/100)
            maximumExtensionPenalty = 1000000;
        else maximumExtensionPenalty = 0;
    end
end

penalty = W_totalTorque * totalTorque + ...
          W_totalqdot * totalqdot     + ...
          W_totalPower * totalPower   + ...
          W_maxTorque * maxTorque     + ...
          W_maxqdot * maxqdot         + ...
          W_maxPower * maxPower       + ...
          trackingErrorPenalty + ...
          jointBelowEEPenalty + ...
          maximumExtensionPenalty;
end
