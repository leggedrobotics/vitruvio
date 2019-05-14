%% getInverseDynamics

function jointTorque = getInverseDynamics(EEselection, q, meanCyclicMotionHipEE, robotConfig, config)
    for i = 1:length(q.(EEselection).angAccel)
        jointVel = q.(EEselection).angVel(i,1:3);
        jointAccel = q.(EEselection).angAccel(i,:);

        wrench = [0 0 0 meanCyclicMotionHipEE.(EEselection).force(i,1:3)]; % need to rotate this force to base frame?
        fext = externalForce(robotConfig,'body4',wrench); % apply force on body4 = end effector

        jointTorque(i,:) = inverseDynamics(robotConfig, config(i,:), jointVel, jointAccel, fext);

    end
end