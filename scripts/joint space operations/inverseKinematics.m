function [jointPositions, r1, r2, r3, r4, r5, rEE] = inverseKinematics(linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);

 % Input: desired end-effector position, quadruped properties
 %        initial guess for joint angles, threshold for the stopping-criterion
 % Output: joint angles which match desired end-effector position

 %% Setup
  tol = 0.01;
  it = 0;
  r_H_IEE_des = meanCyclicMotionHipEE.(EEselection).position; % desired EE position
  max_it = 1000;

  %% Get initial guess q0 for desired configuration
  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection);
  q = [q0']; % final term will be used for body rotation about inertial frame y
  jointPositions = zeros(length(meanCyclicMotionHipEE.(EEselection).position(:,1)),linkCount+2);
  lambda = 0.001; % damping factor -> values below lambda are set to zero in matrix inversion
  % Initialize error -> only position because we don't have orientation data
  rotBodyY = 0;
  q = zeros(linkCount+2, 1);
  [J_P, C_IEE, r_H_I1, r_H_I2, r_H_I3, r_H_I4, r_H_I5, r_H_IEE] = jointToPosJac(linkCount, rotBodyY, q, quadruped, EEselection);
  
  % preallocate arrays for joint coordinates
  r1 = zeros(length(meanCyclicMotionHipEE.(EEselection).position(:,1)), 3);
  r2 = r1;
  r3 = r1;
  r4 = r1;
  r5 = r1;
  r6 = r1;

  %% Iterative inverse kinematics
  for i = 1:length(meanCyclicMotionHipEE.(EEselection).position(:,1))
       % to keep system right handed, input body rotation as negative. A
       % negative value then means positive angle of attack.
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(i,2); % rotation of body about inertial y
        it = 0; % reset iteration count
        dr = r_H_IEE_des(i,:)' - r_H_IEE;
        
      while (norm(dr)>tol && it < max_it)
         [J_P, C_IEE, r_H_I1, r_H_I2, r_H_I3, r_H_I4, r_H_I5, r_H_IEE] = jointToPosJac(linkCount, rotBodyY, q, quadruped, EEselection);
         dr = r_H_IEE_des(i,:)' - r_H_IEE;
         dq = pinv(J_P, lambda)*dr;
          q = q + 0.002*dq; % keep update size small to prevent overshooting angle
         it = it+1;    
      end  
      
      %fprintf('Inverse kinematics terminated after %d iterations.\n',it);
      jointPositions(i,:) = q';
      rEE(i,:) = r_H_IEE;
      % x y z coordinates of joints
      r1(i,:) = r_H_I1;
      r2(i,:) = r_H_I2;
      r3(i,:) = r_H_I3;
      r4(i,:) = r_H_I4;
      r5(i,:) = r_H_I5;
  end     