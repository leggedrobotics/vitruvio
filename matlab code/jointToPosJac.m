function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.
  
  % Compute the relative homogeneous transformation matrices.
%   T_I0 = getTransformI0();

  T_HipHAA = [1, 0,  0;
              0, cos(q(1)),   -sin(q(1));
              0,         sin(q(1)),           cos(q(1))];

  T_HAA1 = [cos(q(2)), -sin(q(2)),  0;
            sin(q(2)), cos(q(2)),   0;
            0,         0,           1];
        
  T_12 =   [cos(q(3)), -sin(q(3)),  0;
            sin(q(3)), cos(q(3)),   0;
            0,         0,           1];
        
  T_23 =   [cos(q(4)), -sin(q(4)),   0;
            sin(q(4)),  cos(q(4)),   0;
            0,          0,           1];
  
  % Compute the homogeneous transformation matrices from frame k to the
  % inertial frame I.
  T_Hip1 = T_HipHAA*T_HAA1
  T_Hip2 = T_Hip1*T_12;
  T_Hip3 = T_Hip2*T_23;
    
%   % Extract the rotation matrices from each homogeneous transformation
%   % matrix.
%   R_I1 = T_I1(1:3,1:3);
%   R_I2 = T_I2(1:3,1:3);
%   R_I3 = T_I3(1:3,1:3);
%   R_I4 = T_I4(1:3,1:3);
%   R_I5 = T_I5(1:3,1:3);
%   R_I6 = T_I6(1:3,1:3);
   
  % Extract the position vectors from each homogeneous transformation
  % matrix.
%   r_I_I1 = T_I1(1:3,4);
%   r_I_I2 = T_I2(1:3,4);
%   r_I_I3 = T_I3(1:3,4);
%   r_I_I4 = T_I4(1:3,4);
%   r_I_I5 = T_I5(1:3,4);
%   r_I_I6 = T_I6(1:3,4);
  
  % Define the unit vectors around which each link rotates in the precedent
  % coordinate frame.
  n_HAA = [1 0 0]';
  n_1 = [0 0 1]';
  n_2 = [0 0 1]';
  n_3 = [0 0 1]';

  
  % Compute the end-effector position vector.
  Hip_r_HipHAA = T_HipHAA*[0; 0; 0];
  Hip_r_HAA1 =  T_HAA1*[l_hip; 0; 0];
  Hip_r_12 =  T_12*[l_thigh; 0; 0];
  Hip_r_23 =  T_23*[l_shank; 0; 0];


  Hip_r_Hip_EE = ;
  
  % Compute the translational jacobian.
  J_P = [   cross(R_I1*n_1, r_I_IE - r_I_I1) ...
            cross(R_I2*n_2, r_I_IE - r_I_I2) ...
            cross(R_I3*n_3, r_I_IE - r_I_I3) ...
            cross(R_I4*n_4, r_I_IE - r_I_I4) ...
            cross(R_I5*n_5, r_I_IE - r_I_I5) ...
            cross(R_I6*n_6, r_I_IE - r_I_I6) ...
        ];
  
end