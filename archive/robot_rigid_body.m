%% Read in data for quadruped geometry
function [] = getRobotRigidBodyModel(quadruped, q) 
l_hip = 0.05;
l_thigh = 0.25;
l_shank = 0.2;
nominal_hip_x = 0.4;
nominal_hip_y = 0.2;

% joint limits
hip_flexion_nominal = 0;
hip_extension_nominal = 0;
knee_extension_nominal = 0;

hip_flexion_limit = pi/8;
hip_extension_limit = pi/8;
knee_extension_limit = pi/8;


% d  offset along previous z to the common normal
% θ angle about previous z from old x {\displaystyle x} x to new x 
% r Assuming a revolute joint, this is the radius about previous z.
% α angle about common normal, from old z to new z axis

%% Build quadruped rigid body model using Denavit Hartenberg parameters in home config
dhparams = [ 0            -pi/2   0              0  
    
             nominal_hip_x 0      nominal_hip_y  0
             0             pi/2	  l_hip          0
            -l_thigh       0     -l_thigh        0
             l_shank	   0     -l_shank        0
             
             nominal_hip_x 0     -nominal_hip_y  0
             0             pi/2	 -l_hip          0
            -l_thigh       0     -l_thigh        0
             l_shank	   0     -l_shank        0
             
            -nominal_hip_x 0      nominal_hip_y  0
             0             pi/2	  l_hip          0
             l_thigh       0     -l_thigh        0
            -l_shank	   0     -l_shank        0
             
            -nominal_hip_x 0     -nominal_hip_y  0
             0             pi/2	 -l_hip          0
             l_thigh       0     -l_thigh        0
            -l_shank	   0     -l_shank        0];

%% Create and assemble rigid bodies

% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree;

body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','fixed');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')

%% LF leg
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','fixed');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');

%% RF leg
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','fixed');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
body8 = robotics.RigidBody('body8');
jnt8 = robotics.Joint('jnt8','revolute');
body9 = robotics.RigidBody('body9');
jnt9 = robotics.Joint('jnt9','revolute');

%% RF leg
body10 = robotics.RigidBody('body10');
jnt10 = robotics.Joint('jnt10','fixed');
body11 = robotics.RigidBody('body11');
jnt11 = robotics.Joint('jnt11','revolute');
body12 = robotics.RigidBody('body12');
jnt12 = robotics.Joint('jnt12','revolute');
body13 = robotics.RigidBody('body13');
jnt13 = robotics.Joint('jnt13','revolute');

%% RH leg
body14 = robotics.RigidBody('body14');
jnt14 = robotics.Joint('jnt14','fixed');
body15 = robotics.RigidBody('body15');
jnt15 = robotics.Joint('jnt15','revolute');
body16 = robotics.RigidBody('body16');
jnt16 = robotics.Joint('jnt16','revolute');
body17 = robotics.RigidBody('body17');
jnt17 = robotics.Joint('jnt17','revolute');


setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');
setFixedTransform(jnt8,dhparams(8,:),'dh');
setFixedTransform(jnt9,dhparams(9,:),'dh');
setFixedTransform(jnt10,dhparams(10,:),'dh');
setFixedTransform(jnt11,dhparams(11,:),'dh');
setFixedTransform(jnt12,dhparams(12,:),'dh');
setFixedTransform(jnt13,dhparams(13,:),'dh');
setFixedTransform(jnt14,dhparams(14,:),'dh');
setFixedTransform(jnt15,dhparams(15,:),'dh');
setFixedTransform(jnt16,dhparams(16,:),'dh');
setFixedTransform(jnt17,dhparams(17,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
body8.Joint = jnt8;
body9.Joint = jnt9;
body10.Joint = jnt10;
body11.Joint = jnt11;
body12.Joint = jnt12;
body13.Joint = jnt13;
body14.Joint = jnt14;
body15.Joint = jnt15;
body16.Joint = jnt16;
body17.Joint = jnt17;


addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body1')
addBody(robot,body7,'body6')
addBody(robot,body8,'body7')
addBody(robot,body9,'body8')
addBody(robot,body10,'body1')
addBody(robot,body11,'body10')
addBody(robot,body12,'body11')
addBody(robot,body13,'body12')
addBody(robot,body14,'body1')
addBody(robot,body15,'body14')
addBody(robot,body16,'body15')
addBody(robot,body17,'body16')

%% Change joint axis for hip and knee extension to y axis and hip flexion to x axis
%extension joints
jnt3.JointAxis = [0 1 0];
jnt4.JointAxis = [0 1 0];
jnt5.JointAxis = [0 1 0];

jnt7.JointAxis = [0 1 0];
jnt8.JointAxis = [0 1 0];
jnt9.JointAxis = [0 1 0];

jnt11.JointAxis = [0 1 0];
jnt12.JointAxis = [0 1 0];
jnt13.JointAxis = [0 1 0];

jnt15.JointAxis = [0 1 0];
jnt16.JointAxis = [0 1 0];
jnt17.JointAxis = [0 1 0];

%flexion joints
% jnt2.JointAxis = [1 0 0];
% jnt6.JointAxis = [1 0 0];
% jnt10.JointAxis = [1 0 0];
% jnt14.JointAxis = [1 0 0];

%% Set joint limits
%extension joints
jnt3.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
jnt4.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
jnt7.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
jnt8.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
jnt11.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
jnt12.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];
jnt15.PositionLimits = hip_extension_nominal + [-hip_extension_limit hip_extension_limit];
jnt16.PositionLimits = knee_extension_nominal + [-knee_extension_limit knee_extension_limit];

%flexion joints
% jnt2.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt6.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt10.PositionLimits = [-hip_flexion_limit hip_flexion_limit];
% jnt14.PositionLimits = [-hip_flexion_limit hip_flexion_limit];

%% Display robot and details

showdetails(robot)
figure(1)
show(robot);

randConfig = robot.randomConfiguration
figure(2)
show(robot,randConfig);
homeConfig = robot.homeConfiguration
