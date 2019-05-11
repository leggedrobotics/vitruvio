%% getInverseDynamics

function jointTorque = getInverseDynamics(EE, q, meanCyclicMotionHipEE, robotConfig, config)

EE_names = fieldnames(EE);

for j = 1:length(EE_names)
    for i = 1:length(q.(EE_names{j}).angAccel(:,1))
       jointVel = q.(EE_names{j}).angVel(i,1:3);
       jointAccel = q.(EE_names{j}).angAccel(i,:);

       wrench = [0 0 0 meanCyclicMotionHipEE.(EE_names{j}).force(i,1:3)]; % need to rotate this force to base frame?
       fext = externalForce(robotConfig,'body4',wrench);

       jointTorque.(EE_names{j})(i,:) = inverseDynamics(robotConfig, config(i,:), jointVel, jointAccel); %, fext);
    end
end