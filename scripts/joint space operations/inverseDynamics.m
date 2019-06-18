% input EE forces and joint position which defines robot stance at each 
% time step, joint speed and acceleration. Output joint torques required to
% achieve the specified motion given the EE forces

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
        rigidBodyModel = Leg.(EEselection).rigidBodyModel;
        wrench = [0 0 0 meanCyclicMotionHipEE.(EEselection).force(i,1:3)]; % torques and forces applied to the body [Tx Ty Tz Fx Fy Fz]
        fext = externalForce(rigidBodyModel,endEffector,wrench); % apply force on end effector
        jointTorque(i,:) = inverseDynamics(rigidBodyModel, [rotBodyY q], [0 qdot], [0 qdotdot], fext);
    end
% the first term is due to body rotation but this is not related to an actuated joint so we neglect it    
jointTorque = jointTorque(:,2:end);
end
