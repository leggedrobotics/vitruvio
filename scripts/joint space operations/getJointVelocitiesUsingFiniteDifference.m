%% get joint velocities

function [qdot qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt)

q = Leg.(EEselection).q;
for i =1:length(q)-1
    qdot(i,:) = (q(i+1,:) - q(i,:)) /dt;
end 
for i =1:length(qdot)-1
    qdotdot(i,:) = (qdot(i+1,:) - qdot(i,:)) /dt;
end  

qdot = qdot(1:end-1,:); 
qdotdot;
end




