function penalty = runFastJointTorqueSim(quadruped, linkLengths, selectFrontHind, taskSelection, EE, dt, configSelection, EEselection, jointCount, meanCyclicMotionHipEE)
%% Get quadruped properties 
% Convert from cm to m and save lengths
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;

% Compute mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1);
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2);
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3);

%% Inverse kinematics to calculate joint angles for each leg joint
tempLeg.(EEselection).q = inverseKinematics(meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);

%% Get joint positions and EE position from forward dynamics. Used in penalty term.
tempLeg.(EEselection).r = getJointPositions(quadruped, tempLeg, jointCount, EEselection);

%% Build robot model with configuration method - required for inverse dynamics solver
numberOfLoopRepetitions = 1;
viewVisualization = 0;
tempLeg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(quadruped, tempLeg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% The joint accelerations are then computed using finite difference
[tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, tempLeg, quadruped, dt);

%% Get joint torques using inverse dynamics
tempLeg.(EEselection).jointTorque = getInverseDynamics(EEselection, tempLeg, meanCyclicMotionHipEE);

%% Penalty term for ga
errorPositionEE = norm(meanCyclicMotionHipEE.(EEselection).position-tempLeg.(EEselection).r.EE);
penalty = (1/100)*sum(sum(abs(tempLeg.(EEselection).q(1:end-1,1:3) .* tempLeg.(EEselection).jointTorque))) + 0*(1/100)*sum(sum((tempLeg.(EEselection).qdot).^2)) + 0*(1/100)*sum(sum((tempLeg.(EEselection).jointTorque).^2)) + 1000*errorPositionEE;
end
