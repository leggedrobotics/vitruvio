%% get analytical Jacobian RH leg

% takes as input only the current joint angles for the leg in question

function [J_eA_RH, chi_RH] = getAnalyticalJac(q1,q2,q3,q4,quadruped)

%i = 4; % RH leg
j = 2;

%position of end effector relative to hip attachment point

x(i) =     quadruped.length.thigh(j) * cos(q2) ...
         + quadruped.length.shank(j) * cos(q2 + q3) ... 
         + quadruped.length.foot(j)  * cos(q2 + q3 + q4)
      
y(i) =   - quadruped.length.hip(j) * sin(q1) ...
         + (quadruped.length.thigh(j) + quadruped.length.shank(j) + quadruped.length.foot(j)) * sin(pi()/2-q1)
         
z(i) = - quadruped.length.hip(j) * cos(q1)- quadruped.length.thigh(j) * sin(q2) 
                                             - quadruped.length.shank(j) * sin(q2 + q3)
                                             - quadruped.length.foot(j)  * sin(q2 + q3 + q4)
                                             

chi_RH = [x(i); y(i); z(i)];

J_eA_RH = [0, ... 
    - quadruped.length.shank(j)*sin(q2 + q3) - quadruped.length.thigh(j)*sin(q2(i)) - quadruped.length.foot(j)*sin(q2(i) + q3(i) + q4(i)), ...
    - quadruped.length.shank(j)*sin(q2 + q3) - quadruped.length.foot(j)*sin(q2(i) + q3(i) + q4(i)), ...
    - quadruped.length.foot(j)*sin(q2 + q3 + q4); ...
    
    - quadruped.length.hip(j)*cos(q1) - cos(q1-pi/2)*(quadruped.length.foot(j) + quadruped.length.shank(j) + quadruped.length.thigh(j)), 0, 0, 0; ...
    
      quadruped.length.hip(j)*sin(q1),...
    - quadruped.length.shank(j)*cos(q2 + q3) - quadruped.length.thigh(j)*cos(q2) - quadruped.length.foot(j)*cos(q2 + q3 + q4), ...
    - quadruped.length.shank(j)*cos(q2 + q3) - quadruped.length.foot(j)*cos(q2 + q3 + q4), ...
    - quadruped.length.foot(j)*cos(q2 + q3 + q4)];
end
