% Input the robot defined in buildRigidBodyModel. Neglect the forces and
% only compute the dynamic contribution of the torque.

function jointTorqueSwing = getSwingJointTorques(EEselection, Leg, meanCyclicMotionHipEE, linkCount)

    if linkCount == 2
        endEffector = 'body4';
    elseif linkCount == 3
        endEffector = 'body5';
    elseif linkCount == 4
        endEffector = 'body6';
    end
    
    jointTorqueSwing = zeros(length(Leg.(EEselection).qdotdot),linkCount+2);

    for i = 1:length(Leg.(EEselection).qdotdot)
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2);
        q = Leg.(EEselection).q(i,1:linkCount+1);
        qdot = Leg.(EEselection).qdot(i,1:linkCount+1);
        qdotdot = Leg.(EEselection).qdotdot(i,1:linkCount+1);
        
        if Leg.(EEselection).force(i,3) > 0 % Foot is in stance.
           jointTorqueSwing(i,:) = zeros(1,linkCount+2);
        else
           rigidBodyModel = Leg.(EEselection).rigidBodyModelSwing;
           % From Robotics Systems Toolbox: inverseDynamics(robot,configuration,jointVel,jointAccel,fext)
           jointTorqueSwing(i,:) = inverseDynamics(rigidBodyModel, [rotBodyY q], [0 qdot], [0 qdotdot]);
       end
    end
    
    % The first term is due to body rotation but this is not related to an actuated joint so we neglect it
    jointTorqueSwing = jointTorqueSwing(:,2:end);
end