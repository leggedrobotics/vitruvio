% Input the robot defined in buildRigidBodyModel. Compute external forces from
% wrench - note that as we have no moment exerted on the end effectors, the
% external forces are the same as the input end effector forces but ordered into an fext matrix. 
% Compute joint torque for the given joint angles, velocities, acclerations 
% and force at each timestep.

function jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE, linkCount)
if linkCount == 2
    endEffector = 'body4';
elseif linkCount == 3
    endEffector = 'body5';
elseif linkCount == 4
    endEffector = 'body6';
end

% jointTorque = zeros(length(Leg.(EEselection).qdotdot),linkCount+1);
jointTorque = zeros(length(Leg.(EEselection).qdotdot),linkCount+2);
    for i = 1:length(Leg.(EEselection).qdotdot)
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2);
        q = Leg.(EEselection).q(i,1:linkCount+1);
        qdot = Leg.(EEselection).qdot(i,1:linkCount+1);
        qdotdot = Leg.(EEselection).qdotdot(i,1:linkCount+1);
        rigidBodyModel = Leg.(EEselection).rigidBodyModel;
        wrench = [0 0 0 meanCyclicMotionHipEE.(EEselection).force(i,1:3)]; % torques and forces applied to the body [Tx Ty Tz Fx Fy Fz]
        fext = externalForce(rigidBodyModel,endEffector,wrench); % apply force on end effector
        
        % inverse dynamics solver of form:
        % jointTorq = inverseDynamics(robot,configuration,jointVel,jointAccel,fext) computes joint torques for the specified joint configuration, velocities, accelerations, and external forces. 
        jointTorque(i,:) = inverseDynamics(rigidBodyModel, [rotBodyY q], [0 qdot], [0 qdotdot], fext);
%         jointTorque(i,:) = inverseDynamics(rigidBodyModel, q, qdot, qdotdot, fext);
    end
    
% the first term is due to body rotation but this is not related to an actuated joint so we neglect it    
jointTorque = jointTorque(:,2:end);

end