function jointPositions = inverseKinematics(meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);

 % Input: desired end-effector position, quadruped properties
 %        initial guess for joint angles, threshold for the stopping-criterion
 % Output: joint angles which match desired end-effector position

 %% Setup
  tol = 0.0001;
  it = 0;
  r_H_HEE_des = meanCyclicMotionHipEE.(EEselection).position; % desired EE position
  max_it = 1000;

  %% Get initial guess q0 for desired configuration
  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection);
  q = q0';  
  jointPositions = zeros(length(meanCyclicMotionHipEE.(EEselection).position(:,1)),4);
  lambda = 0.001; % damping factor
  % Initialize error - only position because we don't have orientation data
  [J_P, C_HEE, r_H_HEE, T_H1, T_12, T_23, T_34] = jointToPosJac(q, quadruped, EEselection);
  
  %% Iterative inverse kinematics
  for i = 1:length(meanCyclicMotionHipEE.(EEselection).position(:,1))
        it = 0; % reset iteration count
        dr = r_H_HEE_des(i,:)' - r_H_HEE;
        
      while (norm(dr)>tol && it < max_it)
         [J_P, C_HEE, r_H_HEE] = jointToPosJac(q, quadruped, EEselection);
         dr = r_H_HEE_des(i,:)' - r_H_HEE;
         dq = pinv(J_P, lambda)*dr;
         q = q + 0.05*dq; % keep update size small to prevent overshooting angle
         it = it+1;    
      end   
%       fprintf('Inverse kinematics terminated after %d iterations.\n',it);
      jointPositions(i,:) = q';
  end     