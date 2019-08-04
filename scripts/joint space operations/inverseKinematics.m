function [jointPositions, r1, r2, r3, r4, r5, rEE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, taskSelection, configSelection, hipParalleltoBody)

 % Input: desired end-effector position, quadruped properties
 %        initial guess for joint angles, threshold for the stopping-criterion
 % Output: joint angles which match desired end-effector position

 %% Setup
  tol = 0.000001; % [m] 
  % When this value is too large, oscillations may be 
  % obtained in the velocity data as the joints do not need to move to bring
  % the error down below the tolerance from one step to another then 
  % eventually they will suddenly shift as the error exceeds the tolerance.
  it = 0;
  r_H_0EE_des = meanCyclicMotionHipEE.(EEselection).position; % desired EE position

  %% Initialize IK algorithm
  q0 = getInitialJointAnglesForDesiredConfig(EEselection, configSelection);
  q = [q0'];
  jointPositions = zeros(length(meanCyclicMotionHipEE.(EEselection).position(:,1)),linkCount+2); % initialize with zeros

  % AFE heuristic
  if linkCount == 3
      if heuristic.torqueAngle.apply == true
          q = qLiftoff.(EEselection);
          jointPositions(1,:) = qLiftoff.(EEselection); % first position already solved when heuristic applied
      else
          q = [q; 0];
      end
  end
  
  % AFE and DFE heuristics
  if linkCount == 4
      if heuristic.torqueAngle.apply == true
          q = qLiftoff.(EEselection);
          jointPositions(1,:) = qLiftoff.(EEselection); % first position already solved when heuristic applied         
      else
           q = [q; 0; 0];
      end
  end
 
  % Initialize error
  rotBodyY = 0;
  [~, ~, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, q, robotProperties, EEselection, hipParalleltoBody);
  
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
        rotBodyY = 0*(-meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2)); % rotation of body about inertial y
        it = 0; % reset iteration count
        lambda = 0.001; % damping factor -> values below lambda are set to zero in matrix inversion
        dr = r_H_0EE_des(i,:)' - r_H_0EE; % error vector
        springDeformation = 0;
        
        if i < 2 % fine update for first point, then can make update more coarse
            k = 0.001; % scale update step to prevent overshoot of desired point
            max_it = 10000;
        else 
            k = 0.2;
            max_it = 800;
        end 
        
        % AFE heuristic (model torsional spring at AFE joint)
         if linkCount > 2
            if i > 1 % first time step has already been solved
                if heuristic.torqueAngle.apply == true
                    qPrevious = jointPositions(i-1,:); % use joint angles from last time step to compute the joint deformation at the current time step
                    EE_force = meanCyclicMotionHipEE.(EEselection).force(i,1:3);
                    [~, springDeformation] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, robotProperties, EEselection, hipParalleltoBody);
                    q(4) = jointPositions(1,4) + springDeformation; % qAFE is it's initial undeformed value + spring deformation due to torque at previous timestep                     
                end
            end
         end 
         
         % iterative IK until error within tolerance or max iterations
         % exceeded
          while (norm(dr)>tol && it < max_it)
             [J_P, ~, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, q, robotProperties, EEselection, hipParalleltoBody);
             dr = r_H_0EE_des(i,:)' - r_H_0EE;
             dq = pinv(J_P, lambda)*dr;
             q = q + k*dq;
             it = it+1;    
             q(4) = jointPositions(1,4) + springDeformation; % hold qAFE at this value
             
             % qAFE and qDFE heuristic (foot and thigh links parallel, qDFE modelled as torsional spring)
             if heuristic.torqueAngle.apply == true
                if i > 1
                    if linkCount == 4
                         q(4) = -q(3); % foot parallel to thigh requires qAFE = -qKFE 
                         q(5) = jointPositions(1,5) + springDeformation; % initial value of the joint angle plus spring deformation
                     end
                 end
             end 
          end  

          jointPositions(i,:) = q';
          rEE(i,:) = r_H_0EE; %% EE coordinates
          % x y z coordinates of each joint
          r1(i,:) = r_H_01;
          r2(i,:) = r_H_02;
          r3(i,:) = r_H_03;
          r4(i,:) = r_H_04;
          r5(i,:) = r_H_05;
  end 
  % We can remove the final two points that were looped around. This
  % ensures the position, vel, accel vectors all have the same length after
  % applying finite difference and the data exactly captures one cycle.
  rEE = rEE(1:end-2,:);
  r1 = r1(1:end-2,:);
  r2 = r2(1:end-2,:);
  r3 = r3(1:end-2,:);
  r4 = r4(1:end-2,:);
  r5 = r5(1:end-2,:);  