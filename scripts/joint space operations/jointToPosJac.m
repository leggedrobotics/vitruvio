function [J_P, C_0EE, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE]  = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, q, quadruped, EEselection, hipParalleltoBody)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in hip attachmemt frame.
  
  if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
    hipOffsetDirection = 1;
  else
    selectFrontHind = 2;
    hipOffsetDirection = -1;
  end
    
  % Compute the relative homogeneous transformation matrices.
  l_hip = quadruped.hip(selectFrontHind).length;
  l_thigh = quadruped.thigh(selectFrontHind).length;
  l_shank = quadruped.shank(selectFrontHind).length;
  l_foot = quadruped.foot(selectFrontHind).length;
  l_phalanges = quadruped.phalanges(selectFrontHind).length;

  
  % rotation about y of inertial frame to create hip attachment frame
  T_0H = [cos(rotBodyY), 0, sin(rotBodyY), 0;
         0,              1, 0,             0;
        -sin(rotBodyY),  0, cos(rotBodyY), 0;
         0,              0, 0,             1];
     
  % transformation from hip attachment frame to HAA frame   
  % rotation about x of hip attachment frame (HAA rotation)
  T_H1 = [1, 0,          0,          hipAttachmentOffset;
          0, cos(q(1)), -sin(q(1)),  0;
          0, sin(q(1)),  cos(q(1)),  0;
          0, 0,          0,          1];

  % transformation from HFE to HAA
  % rotation about y, translation along hip link
% 
if hipParalleltoBody == true
    T_12 = [cos(q(2)), 0,  sin(q(2)),  hipOffsetDirection*l_hip;
            0,         1,  0,          0;
           -sin(q(2)), 0,  cos(q(2)),  0;
            0,         0,  0,          1];  
else
    T_12 = [cos(q(2)), 0,  sin(q(2)),  0;
            0,         1,  0,          0;
           -sin(q(2)), 0,  cos(q(2)),  l_hip;
            0,         0,  0,          1];   
end   
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
  T_01 = T_0H*T_H1;
  T_02 = T_01*T_12;
  T_03 = T_02*T_23;
  T_04 = T_03*T_34; %EE for 2 link leg
  
  if (linkCount == 3) | (linkCount == 4)
      T_05 = T_04*T_45;   %EE for 3 link leg
  end
  
  if linkCount == 4
      T_06 = T_05*T_56; % EE for 4 link leg
  end
  
  % Extract the rotation matrices from each homogeneous transformation
  % matrix.
  R_0H = T_0H(1:3,1:3);
  R_01 = T_01(1:3,1:3);
  R_02 = T_02(1:3,1:3);
  R_03 = T_03(1:3,1:3);
  R_04 = T_04(1:3,1:3);
  if (linkCount == 3) || (linkCount == 4)
      R_05 = T_05(1:3,1:3);
  end
  if (linkCount == 4)
      R_06 = T_06(1:3,1:3);
  end

  % Extract the position vectors from each homogeneous transformation
  % matrix in inertial frame from origin (hip attachment point) to joint k
  r_H_0H = T_0H(1:3,4); 
  r_H_01 = T_01(1:3,4); % HAA
  r_H_02 = T_02(1:3,4); % HFE
  r_H_03 = T_03(1:3,4); % KFE
  r_H_04 = T_04(1:3,4); % EE or AFE
  r_H_05 = [0; 0; 0]; % zeros if these joints do not exist
  r_H_06 = [0; 0; 0]; 

  if (linkCount == 3) || (linkCount == 4)
      r_H_05 = T_05(1:3,4); %% EE or DFE
  end
  if (linkCount == 4)
      r_H_06 = T_06(1:3,4); % EE
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
      r_H_0EE = r_H_04; 
      C_0EE = T_04(1:3,1:3);
  end
  if (linkCount == 3)
      r_H_0EE = r_H_05; 
      C_0EE = T_05(1:3,1:3);
  end
  if (linkCount == 4)
      r_H_0EE = r_H_06; 
      C_0EE = T_06(1:3,1:3);
  end
  
  % Compute the translational jacobian.
  if (linkCount == 2)
      J_P = [   cross(R_01*n_1, r_H_0EE - r_H_01) ...
                cross(R_02*n_2, r_H_0EE - r_H_02) ...
                cross(R_03*n_3, r_H_0EE - r_H_03) ...
                cross(R_04*n_4, r_H_0EE - r_H_04) ...
             ];
  end
  if (linkCount == 3)
      J_P = [   cross(R_01*n_1, r_H_0EE - r_H_01) ...
                cross(R_02*n_2, r_H_0EE - r_H_02) ...
                cross(R_03*n_3, r_H_0EE - r_H_03) ...
                cross(R_04*n_4, r_H_0EE - r_H_04) ...
                cross(R_05*n_5, r_H_0EE - r_H_05) ...
             ];
  end
  if (linkCount == 4)
      J_P = [   cross(R_01*n_1, r_H_0EE - r_H_01) ...
                cross(R_02*n_2, r_H_0EE - r_H_02) ...
                cross(R_03*n_3, r_H_0EE - r_H_03) ...
                cross(R_04*n_4, r_H_0EE - r_H_04) ...
                cross(R_05*n_5, r_H_0EE - r_H_05) ...
                cross(R_06*n_6, r_H_0EE - r_H_06) ...
             ];
  end
end