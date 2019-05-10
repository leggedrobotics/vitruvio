%% get joint velocities

function joint = getJointVelocitiesUsingJacobian(meanCyclicMotionHipEE, q, quadruped, selectFrontHind, dt)

EE_names = {'LF', 'LH', 'RF', 'RH'};

for j=1:4
for i = 1:length(meanCyclicMotionHipEE.LF.velocity)
    if (EE_names{j} == 'LF') | (EE_names{j} == 'RF')
        selectFrontHind = 1;
    else selectFrontHind = 2;
    end
    
    [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34]  = jointToPosJac(q.LF(i,:), quadruped, selectFrontHind);
    qRotVel.(EE_names{j})(i,:) = inv(J_P(1:3,1:3))* meanCyclicMotionHipEE.LF.velocity(i,:)';
end

% also possible to map accelerations to joint accel using Jacobian?

for i =1:length(qRotVel.LF)-1
    qRotAccel.(EE_names{j})(i,:) = (qRotVel.(EE_names{j})(i+1,:) - qRotVel.(EE_names{j})(i,:)) /dt;
end
  
joint.(EE_names{j}).angle = q.(EE_names{j});
joint.(EE_names{j}).rotationalVelocity = qRotVel.(EE_names{j});
joint.(EE_names{j}).rotationalAcceleration = qRotAccel.(EE_names{j});
end
