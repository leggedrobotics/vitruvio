%% buildRobotRigidBodyModel

%% Read in data for quadruped geometry
function robot = buildRobotRigidBodyModel(quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization) 

%% get quadruped properties for selected end effector  
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    hipOffsetDirection = 1;
    else selectFrontHind = 2;
         hipOffsetDirection = -1;
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

%% Build quadruped rigid body model 
% rotations to align z axis with rotational axis of joint and translations
% along length of link

T_HAA =           [0, 0, 1, 0;
                   0, 1, 0, 0;
                  -1, 0, 0, 0;
                   0, 0, 0, 1];

T_HFEattachment = [1,  0, 0, 0;
                   0,  0, 1, 0;
                   0, -1, 0, hipOffsetDirection*l_hip;
                   0,  0, 0, 1];

T_HFE =           [1, 0, 0, l_thigh;
                   0  1, 0, 0;
                   0, 0, 1, 0;
                   0, 0, 0, 1];

T_KFE =           [1, 0, 0, l_shank;
                   0, 1, 0, 0;
                   0, 0, 1, 0;
                   0, 0, 0, 1];
                   
%% Create and assemble rigid bodies
% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree('DataFormat', 'row');
% Create bodies and joints 
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

% inertia = [Ixx Iyy Izz Iyz Ixz Ixy] relative to body frame in kg/m^2
% note that body3.Inertia encompasses shank and end effector inertia terms
body1.Inertia = [0.000001  1/3*body1.Mass*l_hip^2                             1/3*body1.Mass*l_hip^2                             0 0 0]; %hip      
body2.Inertia = [0.000001  1/3*body2.Mass*l_thigh^2                           1/3*body2.Mass*l_thigh^2                           0 0 0]; %thigh
body3.Inertia = [0.000001  1/3*body3.Mass*l_shank^2+1/2*body4.Mass*l_shank^2  1/3*body3.Mass*l_shank^2+1/2*body4.Mass*l_shank^2  0 0 0]; %shank
body4.Inertia = [0.000001  0.000001                                           0.000001                                           0 0 0]; %EE

% center of mass and mass terms do not affect inertia but are used 
% to compute torque due to gravitational force
body1.CenterOfMass = [0.5*quadruped.hip(selectFrontHind).length   0 0];
body2.CenterOfMass = [0.5*quadruped.thigh(selectFrontHind).length 0 0];
body3.CenterOfMass = [0.5*quadruped.shank(selectFrontHind).length 0 0]; 
body4.CenterOfMass = [0 0 0];

%% set joint transforms - these are only translations, the joint positions are specified in the config array   
setFixedTransform(jnt1, T_HAA);
setFixedTransform(jnt2, T_HFEattachment);
setFixedTransform(jnt3, T_HFE);
setFixedTransform(jnt4, T_KFE);            

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;

%% specify connections between bodies
addBody(robot, body1,'base');
addBody(robot, body2,'body1');
addBody(robot, body3,'body2');
addBody(robot, body4,'body3');

robot.Gravity = [0 0 -9.8];
            
%% Display robot 
for i = 1:length(Leg.(EEselection).q)
    config(i,:) = [Leg.(EEselection).q(i,1), ...
                   Leg.(EEselection).q(i,2), ...
                   Leg.(EEselection).q(i,3)];
end

if viewVisualization
    for j = 1: numberOfLoopRepetitions
        for i = 1:length(Leg.(EEselection).q)
            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
            zlim([-0.8 0.2]);

            figure(11)
            show(robot,config(i,:));

            hold on
            plot3(meanCyclicMotionHipEE.(EEselection).position(:,1), ...
                  meanCyclicMotionHipEE.(EEselection).position(:,2), ...
                  meanCyclicMotionHipEE.(EEselection).position(:,3),'r', 'LineWidth', 3)
            title(EEselection)
            hold off
        end
    end
end

