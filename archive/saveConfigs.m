%% save robot configs
% function [] = robotSingleLegVisualization(quadruped, q, C_IBody, EE, meanCyclicMotionHipEE, EESelection, reachablePositions) 

selectFrontHind = 1;

l_hip = quadruped.hip(selectFrontHind).length;
l_thigh = quadruped.thigh(selectFrontHind).length;
l_shank = quadruped.shank(selectFrontHind).length;

% hip attachment points
xNom.LF = quadruped.xNom(1);
yNom.LF = quadruped.yNom(1);

xNom.LH = -quadruped.xNom(2);
yNom.LH = quadruped.yNom(2);

xNom.RF = quadruped.xNom(1);
yNom.RF = -quadruped.yNom(1);

xNom.RH = -quadruped.xNom(2);
yNom.RH = -quadruped.yNom(2);

zNom = quadruped.zNom; % equal for each hip attachment point

EE_name = fieldnames(EE);



%% Build quadruped rigid body model using 4x4 transformation matrices
% for k = 1:3 % repeat visualization loop
for i = 1:length(q.LF);
    for j = EESelection:EESelection % only displays for selected leg
        
  % transformation from HAA to hip attachment
  % rotation about z of hip attachment
  
  % make rotation matrix fro
  C_IBody(4,:,:) = 0;
  C_IBody(:,4,:) = 0;
  C_IBody(4,4,:) = 1;

  
  
  
%   T_HB.(EE_name{j})(:,:,i) = [1, 0, 0, xNom.(EE_name{j});
%                   0, 1, 0, yNom.(EE_name{j});
%                   0, 0, 1, zNom;
%                   0, 0, 0, 1];
              

%   T_HB.(EE_name{j})(1:3,1:3,i) = cyclicC_IBody(:,:,i);
%   T_HB.(EE_name{j})(4,4,i) = 1;
  T_HB.(EE_name{j}) = eye(4);
  
  T_H1.(EE_name{j})(:,:,i) = [1, 0,          0,          0;
                  0, cos(q.(EE_name{j})(i,1)),   -sin(q.(EE_name{j})(i,1)),  0;
                  0, sin(q.(EE_name{j})(i,1)),    cos(q.(EE_name{j})(i,1)),  0;
                  0, 0,          0,          1];

  % transformation from HFE to HAA
  % rotation about Hy, translation along hip link
  T_12.(EE_name{j})(:,:,i) = [cos(q.(EE_name{j})(i,2)), 0,  sin(q.(EE_name{j})(i,2)),  0;
          0,         1,  0,          0;
         -sin(q.(EE_name{j})(i,2)), 0,  cos(q.(EE_name{j})(i,2)), -l_hip;
          0,         0,  0,          1];   
     
  % transformation from HFE to HAA
  % rotation about Ty, translation along thigh
  T_23.(EE_name{j})(:,:,i) = [cos(q.(EE_name{j})(i,3)), 0,  sin(q.(EE_name{j})(i,3)),  0;
          0,         1,  0,          0;
         -sin(q.(EE_name{j})(i,3)), 0,  cos(q.(EE_name{j})(i,3)), -l_thigh;
          0,         0,  0,          1];   

  % transformation from KFE to EE
  % rotation about Sy, translation along sh  
  T_34.(EE_name{j})(:,:,i) =   [1,    0,   0,   0;
            0,    1,   0,   0;
            0,    0,   1,  -l_shank;
            0,    0,   0,   1];
    end
end



%% Create and assemble rigid bodies

% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree;

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');

setFixedTransform(jnt1,eye(4));
body1.Joint = jnt1;

addBody(robot,body1,'base')

%% LF leg
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute'); % 
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute'); % HAA
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute'); % HFE
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute'); % KFE
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute'); % EE

setFixedTransform(jnt2, T_HB.(EE_name{j})(:,:)); 
setFixedTransform(jnt3, T_H1.(EE_name{j})(:,:,i));
setFixedTransform(jnt4,  T_12.(EE_name{j})(:,:,i));
setFixedTransform(jnt5, T_23.(EE_name{j})(:,:,i));
setFixedTransform(jnt6, T_34.(EE_name{j})(:,:,i));

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

%% Change joint axis for hip and knee extension to y axis and hip flexion to x axis
jnt2.JointAxis = [0 1 0];
jnt3.JointAxis = [1 0 0];
jnt4.JointAxis = [0 1 0];
jnt5.JointAxis = [0 1 0];


%% Set joint limits
%extension joints
% jnt3.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
% jnt4.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
% jnt7.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
% jnt8.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
% jnt11.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
% jnt12.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
% jnt15.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
% jnt16.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];

%flexion joints
% jnt2.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt6.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt10.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt14.PositionLimits = [-hip_flexion_limit hip_flexion_limit];

%% Save configuration
for jointNumber = 1:5
config(1,jointNumber,i) = q(;

%% Add stl file to rigid body

% addVisual(body3,"Mesh",thigh2_universal);

%% Display robot and details

% showdetails(robot);
figure(10)
show(robot);
% addVisual(body3,"Mesh",'thigh2_universal.stl');


hold on
plot3(meanCyclicMotionHipEE.(EE_name{j}).position(:,1),meanCyclicMotionHipEE.(EE_name{j}).position(:,2),meanCyclicMotionHipEE.(EE_name{j}).position(:,3),'r', 'LineWidth', 3)
hold off

xlim([-0.5 0.5]);
ylim([-0.5 0.5]);
zlim([-0.8 0.2]);

% randConfig = robot.randomConfiguration
% figure(2)
% show(robot,randConfig);
% homeConfig = robot.homeConfiguration
