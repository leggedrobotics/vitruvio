%% get joint velocities

function [qRotVel qRotAccel] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, q, quadruped, dt)

for i = 1:length(meanCyclicMotionHipEE.(EEselection).velocity)
    if (EEselection == 'LF') | (EEselection == 'RF')
        selectFrontHind = 1;
    else selectFrontHind = 2;
    end
    
    q_ = q.(EEselection).angle(i,:);
    [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34]  = jointToPosJac(q_, quadruped, selectFrontHind);
    
    % bug in velocity accel torque coming from mean Cyclic Motion Hip. EE
    qRotVel(i,:) = inv(J_P(1:3,1:3))* meanCyclicMotionHipEE.(EEselection).velocity(i,:)';
end

% also possible to map accelerations to joint accel using Jacobian?

for i =1:length(qRotVel)-1
    qRotAccel(i,:) = (qRotVel(i+1,:) - qRotVel(i,:)) /dt;
end  

qRotVel;
qRotAccel;
end




