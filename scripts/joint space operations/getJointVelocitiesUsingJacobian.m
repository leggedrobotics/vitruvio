%% get joint velocities

function [qdot qdotdot] = getJointVelocitiesUsingJacobian(linkCount, EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt)

% for i = 1:length(meanCyclicMotionHipEE.(EEselection).velocity)
%     if (EEselection == 'LF') | (EEselection == 'RF')
%         selectFrontHind = 1;
%     else selectFrontHind = 2;
%     end
%     
%     q_ = Leg.(EEselection).q(i,:);
%     rotBodyY = meanCyclicMotionHipEE.body.eulerAngles(i,2);
%     J_P  = jointToPosJac(linkCount, rotBodyY, q_, quadruped, selectFrontHind);
%     invJ_P = pinv(J_P,0.00001);
%     qdot(i,:) = invJ_P(1:end-1,:)*meanCyclicMotionHipEE.(EEselection).velocity(i,:)';
% end

% also possible to map accelerations to joint accel using Jacobian?

q = Leg.(EEselection).q;

for i =1:length(q)-1
    qdot(i,:) = (q(i+1,:) - q(i,:)) /dt;
end 

for i =1:length(qdot)-1
    qdotdot(i,:) = (qdot(i+1,:) - qdot(i,:)) /dt;
end  

qdot;
qdotdot;
end




