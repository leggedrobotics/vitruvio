%% buildRobotRigidBodyModel

%% Read in data for quadruped geometry
function [robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization) 

%% get quadruped properties for selected end effector
EE_name = fieldnames(EE);

%% Build quadruped rigid body model using 4x4 transformation matrices
  
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    else selectFrontHind = 2;
end

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

% create array of end effector names, user input of end effector selection
% chooses the end effector from this array
EE_name = fieldnames(EE)';

%% Build quadruped rigid body model 
% using transforms to align joints with z axis then configurations 
% to specify rotation angle about z axis

% rotations to align z axis with rotational axis of joint and translations
% along length of link

T_HFEattachment = [1,  0, 0, l_hip;
                   0,  0, 1, 0;
                   0, -1, 0, 0;
                   0,  0, 0, 1];

T_HAA = [0, 0, 1, 0;
         0, 1, 0, 0;
        -1, 0, 0, 0;
         0, 0, 0, 1];

T_HFE = [1, 0, 0, l_thigh;
         0  1, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];

T_KFE = [1, 0, 0, l_shank;
         0, 1, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];
                   
%% Create and assemble rigid bodies

% Create a rigid body tree object to build the robot.
robotConfig = robotics.RigidBodyTree('DataFormat', 'row');

 %% Define bodies and joints

body1 = robotics.RigidBody('body1');
body2 = robotics.RigidBody('body2');
body3 = robotics.RigidBody('body3');
body4 = robotics.RigidBody('body4');

jnt1 = robotics.Joint('jnt1','revolute'); % HAA
jnt2 = robotics.Joint('jnt2','revolute'); % HFE
jnt3 = robotics.Joint('jnt3','revolute'); % KFE
jnt4 = robotics.Joint('jnt4','fixed'); % coordinate system at EE 

body1.Mass = quadruped.hip(selectFrontHind).mass;         
body2.Mass = quadruped.thigh(selectFrontHind).mass;
body3.Mass = quadruped.shank(selectFrontHind).mass;
body4.Mass = quadruped.toe(selectFrontHind).mass;        
            
body1.CenterOfMass = [0.5*quadruped.hip(selectFrontHind).length, 0, 0];
body2.CenterOfMass = [0.5*quadruped.thigh(selectFrontHind).length, 0, 0];
body3.CenterOfMass = [0.5*quadruped.shank(selectFrontHind).length, 0, 0];   
            
%% set joint transforms - these are only translations, the angles are specified in the configuration
         
  setFixedTransform(jnt1, T_HAA)
  setFixedTransform(jnt2, T_HFEattachment);
  setFixedTransform(jnt3, T_HFE); % this needs to be rotation about z
  setFixedTransform(jnt4, T_KFE);            

  body1.Joint = jnt1;
  body2.Joint = jnt2;
  body3.Joint = jnt3;
  body4.Joint = jnt4;

%% specify connections between bodies
addBody(robotConfig,body1,'base');
addBody(robotConfig,body2,'body1');
addBody(robotConfig,body3,'body2');
addBody(robotConfig,body4,'body3');

robotConfig.Gravity = [0 0 -9.8];
            
%% Display robot and details
            


for i = 1:length(q.(EEselection).angle)
   
    config(i,:) = [q.(EEselection).angle(i,1), ...
                q.(EEselection).angle(i,2), ...
                q.(EEselection).angle(i,3)];
end

if viewVisualization == 1
    for j = 1: numberOfLoopRepetitions
        for i = 1:length(q.(EEselection))

            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
            zlim([-0.8 0.2]);

            figure(11)
            show(robotConfig,config(i,:));

            hold on
            plot3(meanCyclicMotionHipEE.(EEselection).position(:,1),meanCyclicMotionHipEE.(EEselection).position(:,2),meanCyclicMotionHipEE.(EEselection).position(:,3),'r', 'LineWidth', 3)
            title(EEselection)
            hold off
        end
    end
end

