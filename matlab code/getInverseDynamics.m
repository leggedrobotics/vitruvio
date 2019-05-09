%% getInverseDynamics

% function jointTorque = getInverseDynamics(joint, meanCyclicMotionHipEE, robot)

for i = 1:length(joint.LF.rotationalAcceleration(:,1))
    jointVel_zero = [0, 0, 0, 0, 0] %joint.LF.rotationalVelocity(i,:), 0];
    jointVel = [0, 0, 0, 0, 0]; %joint.LF.rotationalVelocity(i,:), 0];
    jointAccel = [0, joint.LF.rotationalAcceleration(i,:), 0];
    jointAccel_zero = [0, 0 0 0 0];

    wrench = [0 0 0 meanCyclicMotionHipEE.LH.force(i,1:3)]; % need to rotate this force to base frame?
    fext = externalForce(robot,'body6',wrench);
    
    jointTorque_zero(i,:) = inverseDynamics(robot,robot.homeConfiguration,jointVel,jointAccel_zero, fext);
    jointTorque(i,:) = inverseDynamics(robot,robot.homeConfiguration,jointVel,jointAccel, fext);
    

end

% inverseDynamics(robot, config)
