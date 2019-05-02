%% Inverse Kinematics
% import motion of end effectors relative to hips

% import robot geometric and position data

% order:LF LH RF RH
% order: fore, hind

% q1 = HAA
% q2 = HFE_LF HFE_LH HFE_RF HFE_RH
% q3 = KFE
% q4 = AFE
      
%% Numerical inverse kinematics for one leg

tol = 0.01;

q01 = pi()/2;
q02 = pi()/4;
q03 = -pi()/2;
q04 = pi()/2;

q1(1) = q01;
q2(1) = q02;
q3(1) = q03;
q4(1) = q04;

q = [q01; q02; q03; q04]; 
delta_chi = 1;
count = 1;

j = 2; % looking only at hind legs

% for RH leg
chi_des = foot_RH_bodyframe.position;

for i = 1:1 %length(foot_LF.position(:,1)) %run through all time steps
 while norm(delta_chi) > tol && count < 100
%     [J_eA_RH, chi_RH] = getAnalyticalJac(q1(i), q2(i), q3(i), q4(i),quadruped);


% calculate end effector positions
x(i) =     quadruped.length.thigh(j) * cos(q2) ...
         + quadruped.length.shank(j) * cos(q2 + q3) ... 
         + quadruped.length.foot(j)  * cos(q2 + q3 + q4);
      
y(i) =   - quadruped.length.hip(j) * sin(q1) ...
         + (quadruped.length.thigh(j) + quadruped.length.shank(j) + quadruped.length.foot(j)) * sin(pi()/2-q1);
         
z(i) = - quadruped.length.hip(j) * cos(q1) - quadruped.length.thigh(j) * sin(q2) ...
                                           - quadruped.length.shank(j) * sin(q2 + q3) ...
                                           - quadruped.length.foot(j)  * sin(q2 + q3 + q4);                      
                                           
chi_RH = [x(i) y(i) z(i)];
chi_des(i,:);
delta_chi = chi_des(i,:)-chi_RH;
error = norm(delta_chi)


% anayltical Jacobian

J_eA_RH = [0, ... 
    - quadruped.length.shank(j)*sin(q2 + q3) - quadruped.length.thigh(j)*sin(q2(i)) - quadruped.length.foot(j)*sin(q2(i) + q3(i) + q4(i)), ...
    - quadruped.length.shank(j)*sin(q2 + q3) - quadruped.length.foot(j)*sin(q2(i) + q3(i) + q4(i)), ...
    - quadruped.length.foot(j)*sin(q2 + q3 + q4); ...
    
    - quadruped.length.hip(j)*cos(q1) - cos(q1-pi/2)*(quadruped.length.foot(j) + quadruped.length.shank(j) + quadruped.length.thigh(j)), 0, 0, 0; ...
    
      quadruped.length.hip(j)*sin(q1),...
    - quadruped.length.shank(j)*cos(q2 + q3) - quadruped.length.thigh(j)*cos(q2) - quadruped.length.foot(j)*cos(q2 + q3 + q4), ...
    - quadruped.length.shank(j)*cos(q2 + q3) - quadruped.length.foot(j)*cos(q2 + q3 + q4), ...
    - quadruped.length.foot(j)*cos(q2 + q3 + q4)];

J_eA_pinv = pinv(J_eA_RH);

%calculate error between ee positions based on current joint angles and desired positions

q = q + 0.1*J_eA_pinv*delta_chi'

count = count + 1;

end
end
