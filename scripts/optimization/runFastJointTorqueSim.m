function penalty = runFastJointTorqueSim(quadruped, linkLengths, selectFrontHind, taskSelection, EE, dt, configSelection, EEselection, jointCount, meanCyclicMotionHipEE)
%% get quadruped properties 
% convert from cm to m and save lengths
quadruped.hip(selectFrontHind).length = linkLengths(1)/100;
quadruped.thigh(selectFrontHind).length = linkLengths(2)/100;
quadruped.shank(selectFrontHind).length = linkLengths(3)/100;

% compute mass with assumption of constant density cylinder
quadruped.hip(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.hip(selectFrontHind).radius)^2   * linkLengths(1);
quadruped.thigh(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.thigh(selectFrontHind).radius)^2 * linkLengths(2);
quadruped.shank(selectFrontHind).mass = quadruped.legDensity * pi*(quadruped.shank(selectFrontHind).radius)^2 * linkLengths(3);

%% Inverse kinematics to calculate joint angles for each leg joint
q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);

%% Get joint positions and EE position from forward dynamics. Used in penalty term.
r = getJointPositions(quadruped, q, jointCount, EEselection);

%% Build robot model with configuration method - required for inverse dynamics solver
numberOfLoopRepetitions = 1;
viewVisualization = 0;
[robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% the joint accelerations are then computed using finite difference
[q.(EEselection).angVel, q.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt);

%% Get joint torques using inverse dynamics
jointTorque.(EEselection) = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config);

%% penalty term for ga
errorPositionEE = norm(meanCyclicMotionHipEE.(EEselection).position-r.(EEselection).EE);
penalty = (1/100)*sum(sum(abs(q.(EEselection).angVel(1:end-1,:).*jointTorque.(EEselection)))) + 0*(1/100)*sum(sum((q.(EEselection).angVel).^2)) + 0*(1/100)*sum(sum((jointTorque.(EEselection)).^2)) + 1000*errorPositionEE;
end
