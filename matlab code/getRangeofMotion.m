%% compute range of motion of EE

function [reachablePositionsFront reachablePositionsHind] = getRangeofMotion(quadruped)

l_hip_front = quadruped.hip(1).length;
l_thigh_front = quadruped.thigh(1).length; 
l_shank_front = quadruped.shank(1).length; 

l_hip_hind = quadruped.hip(2).length;
l_thigh_hind = quadruped.thigh(2).length; 
l_shank_hind = quadruped.shank(2).length; 

q1_min = quadruped.q1.minAngle;
q1_max = quadruped.q1.maxAngle;
q2_min = quadruped.q2.minAngle;
q2_max = quadruped.q2.maxAngle;
q3_min = quadruped.q3.minAngle;
q3_max = quadruped.q3.maxAngle;
q4_min = quadruped.q4.minAngle;
q4_max = quadruped.q4.maxAngle;

q1 = 0:0.1:pi/2; % all possible HAA
q2 = -pi/4:0.1:pi/4; % all possible HFE
q3 = -pi/2:0.1:pi/2; % all possible KFE
q4 = q2;

[THETA2,THETA3] = meshgrid(q2,q3); % generate a grid of theta1 and theta2 values

% front legs
X_front =  l_thigh_front * sin(THETA2) + l_shank_front * sin(THETA2 + THETA3); % compute x coordinates
Z_front = -l_hip_front + -l_thigh_front * cos(THETA2) - l_shank_front * cos(THETA2 + THETA3); % compute z coordinates

% hind legs
X_hind =  l_thigh_hind * sin(THETA2) + l_shank_hind * sin(THETA2 + THETA3); % compute x coordinates
Z_hind = -l_hip_hind + -l_thigh_hind * cos(THETA2) - l_shank_hind * cos(THETA2 + THETA3); % compute z coordinates

reachablePositionsFront = [X_front(:) Z_front(:)];
reachablePositionsHind = [X_hind(:) Z_hind(:)];

% data1 = [X(:) Y(:) Z(:) THETA1(:)]; % create x-y-theta1 dataset
% data2 = [X(:) Y(:) Z(:) THETA2(:)]; % create x-y-theta2 dataset