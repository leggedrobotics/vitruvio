function [springTorque, springDeformation] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody)
kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
% compute jacobian to obtain position of AFE, DFE, and EE.
[~, ~, ~, ~, ~, r_H_04, r_H_05, r_H_0EE]  = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, qPrevious, quadruped, EEselection, hipParalleltoBody);

R_EE = r_H_0EE;
if linkCount == 3
    R_finalJoint = r_H_04;
elseif linkCount == 4
    R_finalJoint = r_H_05;
end

R_EEfinalJoint = R_EE - R_finalJoint;
springTorque = cross(R_EEfinalJoint, EE_force);
springTorque = springTorque(1,2); % spring torque about y joint is the relevant torque
springDeformation = springTorque/kTorsionalSpring; 
end