function [J_P, C_IEE, r_H_I1, r_H_I2, r_H_I3, r_H_I4, r_H_I5, r_H_IEE]  = jointToPosJac(linkCount, rotBodyY, q, quadruped, EEselection)
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
  l_foot = quadruped.foot(selectFrontHind).length;
  l_phalanges = quadruped.phalanges(selectFrontHind).length;

  
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

% if there is a third link, overwrite T_34 and compute T_45        
        if (linkCount == 3) | (linkCount == 4)
          % ankle joint q(4)
          T_34 = [cos(q(4)), 0,  sin(q(4)),  l_shank;
                  0,         1,  0,          0;
                 -sin(q(4)), 0,  cos(q(4)),  0;
                  0,         0,  0,          1]; 

          T_45 = [1, 0,  0,  l_foot;
                  0, 1,  0,  0;
                  0, 0,  1,  0;
                  0, 0,  0,  1];
        end
        
% if there is a fourth link, overwrite T_45 and compute T_56        
        if linkCount == 4
          % distal joint q(5)
          T_45 = [cos(q(5)), 0,  sin(q(5)),  l_foot;
                  0,         1,  0,          0;
                 -sin(q(5)), 0,  cos(q(5)),  0;
                  0,         0,  0,          1]; 

           T_56 = [1, 0,  0,  l_phalanges;
                   0, 1,  0,  0;
                   0, 0,  1,  0;
                   0, 0,  0,  1]; 
        end
       
  % Compute the homogeneous transformation matrices from frame k to the
  % inertial frame I
  T_I1 = T_IH*T_H1;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  T_I4 = T_I3*T_34; %EE for 2 link leg
  
  if (linkCount == 3) | (linkCount == 4)
      T_I5 = T_I4*T_45;   %EE for 3 link leg
  end
  
  if linkCount == 4
      T_I6 = T_I5*T_56; % EE for 4 link leg
  end
  
  % Extract the rotation matrices from each homogeneous transformation
  % matrix.
  R_IH = T_IH(1:3,1:3);
  R_I1 = T_I1(1:3,1:3);
  R_I2 = T_I2(1:3,1:3);
  R_I3 = T_I3(1:3,1:3);
  R_I4 = T_I4(1:3,1:3);
  if (linkCount == 3) | (linkCount == 4)
      R_I5 = T_I5(1:3,1:3);
  end
  if (linkCount == 4)
      R_I6 = T_I6(1:3,1:3);
  end

  % Extract the position vectors from each homogeneous transformation
  % matrix in inertial frame from hip attachment point to joint k
  r_H_IH = T_IH(1:3,4); 
  r_H_I1 = T_I1(1:3,4); % HAA
  r_H_I2 = T_I2(1:3,4); % HFE
  r_H_I3 = T_I3(1:3,4); % KFE
  r_H_I4 = T_I4(1:3,4); % EE or AFE
  r_H_I5 = [0; 0; 0]; % zeros if these joints do not exist
  r_H_I6 = [0; 0; 0]; 

  if (linkCount == 3) | (linkCount == 4)
      r_H_I5 = T_I5(1:3,4); %% EE or DFE
  end
  if (linkCount == 4)
      r_H_I6 = T_I6(1:3,4); % EE
  end
  
  % Define the unit vectors around which each link rotates in the precedent
  % coordinate frame.
  n_H = [0 1 0]';
  n_1 = [1 0 0]';
  n_2 = [0 1 0]';
  n_3 = [0 1 0]';
  n_4 = [0 0 0]'; % translation from knee to EE
  if (linkCount == 3)
      n_4 = [0 1 0]'; % rotation about AFE
      n_5 = [0 0 0]'; % translation from ankle to EE
  end
  if (linkCount == 4)
      n_4 = [0 1 0]';
      n_5 = [0 1 0]'; % rotation about distal joint
      n_6 = [0 0 0]'; % translation from distal joint to EE
  end
  
  % return joint to position and joint to rotation matrix for IK algorithm
  if (linkCount == 2)
      r_H_IEE = r_H_I4; 
      C_IEE = T_I4(1:3,1:3);
  end
  if (linkCount == 3)
      r_H_IEE = r_H_I5; 
      C_IEE = T_I5(1:3,1:3);
  end
  if (linkCount == 4)
      r_H_IEE = r_H_I6; 
      C_IEE = T_I6(1:3,1:3);
  end
  
  % Compute the translational jacobian.
  if (linkCount == 2)
      J_P = [   cross(R_I1*n_1, r_H_IEE - r_H_I1) ...
                cross(R_I2*n_2, r_H_IEE - r_H_I2) ...
                cross(R_I3*n_3, r_H_IEE - r_H_I3) ...
                cross(R_I4*n_4, r_H_IEE - r_H_I4) ...
             ];
  end
  if (linkCount == 3)
      J_P = [   cross(R_I1*n_1, r_H_IEE - r_H_I1) ...
                cross(R_I2*n_2, r_H_IEE - r_H_I2) ...
                cross(R_I3*n_3, r_H_IEE - r_H_I3) ...
                cross(R_I4*n_4, r_H_IEE - r_H_I4) ...
                cross(R_I5*n_5, r_H_IEE - r_H_I5) ...
             ];
  end
  if (linkCount == 4)
      J_P = [   cross(R_I1*n_1, r_H_IEE - r_H_I1) ...
                cross(R_I2*n_2, r_H_IEE - r_H_I2) ...
                cross(R_I3*n_3, r_H_IEE - r_H_I3) ...
                cross(R_I4*n_4, r_H_IEE - r_H_I4) ...
                cross(R_I5*n_5, r_H_IEE - r_H_I5) ...
                cross(R_I6*n_6, r_H_IEE - r_H_I6) ...
             ];
  end
end