function [J_P, C_IEE, r_H_IEE]  = jointToPosJac(linkCount, rotBodyY, q, quadruped, EEselection)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in hip attachmemt frame.
  
  if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    hipOffsetDirection = 1;
    else selectFrontHind = 2;
         hipOffsetDirection = -1;
  end
    
  % Compute the relative homogeneous transformation matrices.
  l_hip = quadruped.hip(selectFrontHind).length;
  l_thigh = quadruped.thigh(selectFrontHind).length;
  l_shank = quadruped.shank(selectFrontHind).length;
  
  % rotation about y of inertial frame to create hip attachment frame
  T_IH = [cos(rotBodyY),  0, sin(rotBodyY), 0;
         0,           1, 0,         0;
         -sin(rotBodyY),  0, cos(rotBodyY), 0;
         0,           0, 0,         1];
     
  % transformation from hip attachment frame to HAA frame   
  % rotation about x of hip attachment frame (HAA rotation)
  T_H1 = [1, 0,          0,          0;
         0, cos(q(1)), -sin(q(1)),  0;
         0, sin(q(1)),  cos(q(1)),  0;
         0, 0,          0,          1];

  % transformation from HFE to HAA
  % rotation about y, translation along hip link
  T_12 = [cos(q(2)), 0,  sin(q(2)),  hipOffsetDirection*l_hip;
          0,         1,  0,          0;
         -sin(q(2)), 0,  cos(q(2)),  0;
          0,         0,  0,          1];   
     
  % transformation from HFE to KFE
  % rotation about y, translation along z
  T_23 = [cos(q(3)), 0,  sin(q(3)),  l_thigh;
          0,         1,  0,          0;
         -sin(q(3)), 0,  cos(q(3)),  0;
          0,         0,  0,          1];   

  % transformation from KFE to EE
  % translation along z 
  T_34 =   [1,    0,   0,   l_shank;
            0,    1,   0,   0;
            0,    0,   1,   0;
            0,    0,   0,   1];
            
  % Compute the homogeneous transformation matrices from frame k to the
  % inertial frame I
  T_I1 = T_IH*T_H1;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  T_I4 = T_I3*T_34;
  
  % Extract the rotation matrices from each homogeneous transformation
  % matrix.
  R_IH = T_IH(1:3,1:3);
  R_I1 = T_I1(1:3,1:3);
  R_I2 = T_I2(1:3,1:3);
  R_I3 = T_I3(1:3,1:3);
  R_I4 = T_I4(1:3,1:3);

  % Extract the position vectors from each homogeneous transformation
  % matrix in inertial frame from hip attachment point to joint k
  r_H_IH = T_IH(1:3,4);
  r_H_I1 = T_I1(1:3,4);
  r_H_I2 = T_I2(1:3,4);
  r_H_I3 = T_I3(1:3,4);
  r_H_I4 = T_I4(1:3,4);
  
  % Define the unit vectors around which each link rotates in the precedent
  % coordinate frame.
  n_H = [0 1 0]';
  n_1 = [1 0 0]';
  n_2 = [0 1 0]';
  n_3 = [0 1 0]';
  n_4 = [0 0 0]'; % translation from knee to EE

  % return joint to position and joint to rotation matrix for IK algorithm
  r_H_IEE = r_H_I4; 
  C_IEE = T_I4(1:3,1:3);
  
  % Compute the translational jacobian.
  J_P = [   cross(R_I1*n_1, r_H_IEE - r_H_I1) ...
            cross(R_I2*n_2, r_H_IEE - r_H_I2) ...
            cross(R_I3*n_3, r_H_IEE - r_H_I3) ...
            cross(R_I4*n_4, r_H_IEE - r_H_I4) ...
         ];
  
end