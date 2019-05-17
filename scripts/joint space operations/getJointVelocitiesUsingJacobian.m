%% get joint velocities

function [qdot qdotdot] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt)

for i = 1:length(meanCyclicMotionHipEE.(EEselection).velocity)
    if (EEselection == 'LF') | (EEselection == 'RF')
        selectFrontHind = 1;
    else selectFrontHind = 2;
    end
    
    q_ = Leg.(EEselection).q(i,:);
    [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34]  = jointToPosJac(q_, quadruped, selectFrontHind);
    
    % bug in velocity accel torque coming from mean Cyclic Motion Hip. EE
    qdot(i,:) = inv(J_P(1:3,1:3))* meanCyclicMotionHipEE.(EEselection).velocity(i,:)';
end

% also possible to map accelerations to joint accel using Jacobian?

for i =1:length(qdot)-1
    qdotdot(i,:) = (qdot(i+1,:) - qdot(i,:)) /dt;
end  

qdot;
qdotdot;
end




