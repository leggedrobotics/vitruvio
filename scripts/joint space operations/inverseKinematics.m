function [jointPositions, r1, r2, r3, r4, r5, rEE] = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg)

 % Input: desired end-effector position, quadruped properties
 %        initial guess for joint angles, threshold for the stopping-criterion
 % Output: joint angles which match desired end-effector position

 %% Setup
  tol = 0.0001;
  it = 0;
  r_H_0EE_des = meanCyclicMotionHipEE.(EEselection).position; % desired EE position

  %% Initialize IK algorithm
  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection);
  q = [q0'];
  if linkCount == 3
      if heuristic.torqueAngle.apply == true
          q = qLiftoff.(EEselection);
      else
          q = [q; 0];
      end
  end
  if linkCount == 4
      if heuristic.torqueAngle.apply == true
          q = qLiftoff.(EEselection);
      else
           q = [q; 0; 0];
      end
  end
 
  jointPositions = zeros(length(meanCyclicMotionHipEE.(EEselection).position(:,1)),linkCount+2);
  lambda = 0.001; % damping factor -> values below lambda are set to zero in matrix inversion
  % Initialize error -> only position because we don't have orientation data
  rotBodyY = 0;
  [J_P, ~, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, q, quadruped, EEselection, hipParalleltoBody);
  
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
        dr = r_H_0EE_des(i,:)' - r_H_0EE;
        k = 0.2;
        max_it = 200;
        if i < 2 % fine update for first point, then can make update more coarse
            k = 0.001;
            max_it = 10000;
        end 

      while (norm(dr)>tol && it < max_it)
         [J_P, ~, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(hipAttachmentOffset, linkCount, rotBodyY, q, quadruped, EEselection, hipParalleltoBody);
         dr = r_H_0EE_des(i,:)' - r_H_0EE;
         dq = pinv(J_P, lambda)*dr;
         q = q + k*dq;
         it = it+1;    
         if linkCount == 3
            if i > 1
                if heuristic.torqueAngle.apply == true
                    qPrevious = jointPositions(i-1,:); % use joint angles from last time step to compute the joint deformation at the current time step
                    EE_force = Leg.(EEselection).force(i,1:3);
                    [~, springDeformation] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);
                    q(4) = jointPositions(1,4) + springDeformation;
                end
            end
         end 
         if linkCount == 4
             if i > 1
                 if heuristic.torqueAngle.apply == true
                     qPrevious = jointPositions(i-1,:); % use joint angles from last time step to compute the joint deformation at the current time step
                     EE_force = Leg.(EEselection).force(i,1:3);
                     q(4) = -q(3); % foot parallel to thigh requires qAFE = -qKFE 
                     [~, springDeformation] = computeFinalJointDeformation(heuristic, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);
                     q(5) = jointPositions(1,5) + springDeformation; % initial value of the joint angle plus spring deformation
                 end
             end
         end 
      end  
      
      %fprintf('Inverse kinematics terminated after %d iterations.\n',it);
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
  r1 = r1(1:end-2,:);
  r2 = r2(1:end-2,:);
  r3 = r3(1:end-2,:);
  r4 = r4(1:end-2,:);
  r5 = r5(1:end-2,:);  