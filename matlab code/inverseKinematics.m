 function jointPositions = inverseKinematics(desiredPositionHipEE, q0, quadruped, selectFrontHind)
  % Input: desired end-effector position, desired end-effector orientation (rotation matrix), 
  %        initial guess for joint angles, threshold for the stopping-criterion
  % Output: joint angles which match desired end-effector position and orientation
  
  %% Setup
  tol = 0.0001;
  it = 0;
  r_H_HEE_des = desiredPositionHipEE;
 
  % Set the maximum number of iterations.
  max_it = 1000;

  % Initialize the solution with the initial guess.
  q = q0';  
  jointPositions = zeros(length(desiredPositionHipEE(:,1)),4);

  % Damping factor.
  lambda = 0.001;
  
  % Initialize error - only position because we don't have orientation data
  [J_P, C_HEE, r_H_HEE] = jointToPosJac(q, quadruped, selectFrontHind);
    
  %% Iterative inverse kinematics
  
  % Iterate until terminating condition.
  for i = 1:length(desiredPositionHipEE(:,1))
        it = 0; % reset iteration count
        dr = r_H_HEE_des(i,:)' - r_H_HEE;
        
      while (norm(dr)>tol && it < max_it)
         [J_P, C_HEE, r_H_HEE] = jointToPosJac(q, quadruped, selectFrontHind);
         dr = r_H_HEE_des(i,:)' - r_H_HEE;
         dq = pinv(J_P, lambda)*dr;
         q = q + 0.5*dq;
         it = it+1;
         
      end
      
%       fprintf('Inverse kinematics terminated after %d iterations.\n',it);
%       fprintf('Position error: %e.\n',norm(dr));
      jointPositions(i,:) = q';

  end
 end