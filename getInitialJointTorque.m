%% get joint torque and position, vel, accel for initial link lengths

%% get quadruped properties
quadruped = getQuadrupedProperties(robotSelection);

%% get the relative motion of the end effectors to the hips
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% get the liftoff and touchdown timings for each end effector
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% get the mean cyclic position and forces
[meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody);                  

numberOfLoopRepetitions = 1;
viewVisualization = 0;

initialq.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.(EEselection).position, quadruped, EEselection, taskSelection, configSelection);
[initialRobotConfig, config] = buildRobotRigidBodyModel(quadruped, initialq, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
[initialq.(EEselection).angVel, initialq.(EEselection).angAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, initialq, quadruped, dt);
initialJointTorque.(EEselection) = getInverseDynamics(EEselection, initialq, meanCyclicMotionHipEE, initialRobotConfig, config);
