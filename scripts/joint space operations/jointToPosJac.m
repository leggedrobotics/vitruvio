function [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34]  = jointToPosJac(q, quadruped, EEselection)
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
  
  
  % transformation from HAA to hip attachment
  % rotation about z of hip attachment
  T_H1= [1, 0,          0,          0;
         0, cos(q(1)), -sin(q(1)),  0;
         0, sin(q(1)),  cos(q(1)),  0;
         0, 0,          0,          1];

  % transformation from HFE to HAA
  % rotation about Hy, translation along hip link
  T_12 = [cos(q(2)), 0,  sin(q(2)),  hipOffsetDirection*l_hip;
          0,         1,  0,          0;
         -sin(q(2)), 0,  cos(q(2)),  0;
          0,         0,  0,          1];   
     
  % transformation from HFE to HAA
  % rotation about Ty, translation along thigh
  T_23 = [cos(q(3)), 0,  sin(q(3)),  0;
          0,         1,  0,          0;
         -sin(q(3)), 0,  cos(q(3)), -l_thigh;
          0,         0,  0,          1];   

  % transformation from KFE to EE
  % translation along shank 
  T_34 =   [1,    0,   0,   0;
            0,    1,   0,   0;
            0,    0,   1,  -l_shank;
            0,    0,   0,   1];
            
  % Compute the homogeneous transformation matrices from frame k to the
  % hipattachment frame H
  %T_H1 = T_I0*T_01;
  T_H2 = T_H1*T_12;
  T_H3 = T_H2*T_23;
  T_H4 = T_H3*T_34;
  
  % Extract the rotation matrices from each homogeneous transformation
  % matrix.
  R_H1 = T_H1(1:3,1:3);
  R_H2 = T_H2(1:3,1:3);
  R_H3 = T_H3(1:3,1:3);
  R_H4 = T_H4(1:3,1:3);

  % Extract the position vectors from each homogeneous transformation
  % matrix in hip attachment frame from hip attachment point to joint k
  r_H_H1 = T_H1(1:3,4);
  r_H_H2 = T_H2(1:3,4);
  r_H_H3 = T_H3(1:3,4);
  r_H_H4 = T_H4(1:3,4);
  
  % Define the unit vectors around which each link rotates in the precedent
  % coordinate frame.
  n_1 = [1 0 0]';
  n_2 = [0 1 0]';
  n_3 = [0 1 0]';
  n_4 = [0 0 0]'; % translation from knee to EE

  % return joint to position and joint to rotation matrix for IK algorithm
  r_H_HEE = r_H_H4; 
  C_HEE = T_H4(1:3,1:3);
  
  % Compute the translational jacobian.
  J_P = [   cross(R_H1*n_1, r_H_HEE - r_H_H1) ...
            cross(R_H2*n_2, r_H_HEE - r_H_H2) ...
            cross(R_H3*n_3, r_H_HEE - r_H_H3) ...
            cross(R_H4*n_4, r_H_HEE - r_H_H4) ...
         ];
  
end