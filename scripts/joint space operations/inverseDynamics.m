% Input the robot defined in buildRigidBodyModel. Neglect the forces and
% only compute the dynamic contribution of the torque

function jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE, linkCount)

    if linkCount == 2
        endEffector = 'body4';
    elseif linkCount == 3
        endEffector = 'body5';
    elseif linkCount == 4
        endEffector = 'body6';
    end
    
    jointTorque = zeros(length(Leg.(EEselection).qdotdot),linkCount+2);

    for i = 1:length(Leg.(EEselection).qdotdot)
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2);
        q = Leg.(EEselection).q(i,1:linkCount+1);
        qdot = Leg.(EEselection).qdot(i,1:linkCount+1);
        qdotdot = Leg.(EEselection).qdotdot(i,1:linkCount+1);
       
%         Depending on whether the leg is in stance or swing, the applied
%         gravity is zero or non-zero.
        if Leg.(EEselection).force(i,3) > 0
            rigidBodyModel = Leg.(EEselection).rigidBodyModelStance;
        else
            rigidBodyModel = Leg.(EEselection).rigidBodyModelSwing;
        end
        
        % Inverse dynamics solver inputs: inverseDynamics(robot,configuration,jointVel,jointAccel,fext)
        jointTorque(i,:) = inverseDynamics(rigidBodyModel, [rotBodyY q], [0 qdot], [0 qdotdot]);
    end
    % The first term is due to body rotation but this is not related to an actuated joint so we neglect it    
    jointTorque = jointTorque(:,2:end);
end