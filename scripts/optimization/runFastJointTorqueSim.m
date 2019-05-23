function penalty = runFastJointTorqueSim(linkCount, optimizationProperties, quadruped, linkLengths, selectFrontHind, taskSelection, EE, dt, configSelection, EEselection, jointCount, meanCyclicMotionHipEE)
%% Get quadruped properties 
% Update link lengths, unit in meters
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;

% Update link mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1)/100;
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2)/100;
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3)/100;

%% Inverse kinematics to calculate joint angles for each leg joint
tempLeg.(EEselection).q = inverseKinematics(linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);

%% Get joint positions and EE position from forward dynamics. Used in penalty term.
% tempLeg.(EEselection).r = getJointPositions(quadruped, tempLeg, jointCount, EEselection, meanCyclicMotionHipEE);
% rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(:,2);
% q = tempLeg.(EEselection).q;
% [], [], tempLeg.(EEselection).r.EE = jointToPosJac(rotBodyY, q, quadruped, EEselection);

%% Build robot model with joint angles from inverse kinematics tempLeg
numberOfLoopRepetitions = 1;
viewVisualization = 0;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(linkCount, quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% The joint accelerations are then computed using finite difference
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);

%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = inverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE);

%% Penalty term for ga
W_totalTorque   = optimizationProperties.penaltyWeight.totalTorque;
W_totalqdot     = optimizationProperties.penaltyWeight.totalqdot;
W_totalPower    = optimizationProperties.penaltyWeight.totalPower;
W_maxTorque     = optimizationProperties.penaltyWeight.maxTorque;
W_maxqdot       = optimizationProperties.penaltyWeight.maxqdot;
W_maxPower      = optimizationProperties.penaltyWeight.maxPower;
W_trackingError = optimizationProperties.penaltyWeight.trackingError;

totalTorque   = sum(sum((tempLeg.(EEselection).jointTorque).^2));
totalqdot     = sum(sum((tempLeg.(EEselection).qdot).^2));
totalPower    = sum(sum(abs((tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(1:end-1,:)))));
maxTorque     = max(max(abs(tempLeg.(EEselection).jointTorque)));
maxqdot       = max(max(abs(tempLeg.(EEselection).qdot)));
maxPower      = max(max(abs((tempLeg.(EEselection).jointTorque).*(tempLeg.(EEselection).qdot(1:end-1,:)))));
% trackingError = norm(meanCyclicMotionHipEE.(EEselection).position-tempLeg.(EEselection).r.EE);

penalty = W_totalTorque * totalTorque + ...
          W_totalqdot * totalqdot     + ...
          W_totalPower * totalPower   + ...
          W_maxTorque * maxTorque     + ...
          W_maxqdot * maxqdot         + ...
          W_maxPower * maxPower;
%         W_trackingError * trackingError;

end
