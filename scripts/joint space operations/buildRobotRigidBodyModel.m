%% Read in data for quadruped geometry
function robot = buildRobotRigidBodyModel(l_hipAttachmentOffset, linkCount, quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody) 

%% get quadruped properties for selected end effector  
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
    hipOffsetDirection = 1;
    else selectFrontHind = 2;
         hipOffsetDirection = -1;
end

% offset from nominal stance EE position to HAA along body x
l_hipAttachmentOffsetX = l_hipAttachmentOffset*cos(meanCyclicMotionHipEE.body.eulerAngles(1,2)); 
l_hipAttachmentOffsetZ = l_hipAttachmentOffset*sin(meanCyclicMotionHipEE.body.eulerAngles(1,2));  

l_hip = quadruped.hip(selectFrontHind).length; % offset from HAA to HFE
l_thigh = quadruped.thigh(selectFrontHind).length;
l_shank = quadruped.shank(selectFrontHind).length;
l_foot = quadruped.foot(selectFrontHind).length;
l_phalanges = quadruped.phalanges(selectFrontHind).length;


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

% rotation about x by -pi/2 to align z with inertial y. Rotation about this
% z gives the angle of attack of the base 
T_body =             [1, 0, 0, l_hipAttachmentOffsetX;
                      0, 0, 1,  0;
                      0, -1, 0, l_hipAttachmentOffsetZ;
                      0, 0, 0, 1];
               
% rotation about y by pi/2 to align z with HAA rotation axis
T_HAA =           [0, 0, 1, 0;
                   0, 1, 0, 0;
                  -1, 0, 0, 0;
                   0, 0, 0, 1];

% rotation about y by -pi/2 to align z with HFE rotation axis 
% and hip attachment to HFE translation
if hipParalleltoBody
    T_HFEattachment = [0,  0, -1, 0;
                       0,  1, 0,  0;
                       1,  0, 0, hipOffsetDirection*l_hip;
                       0,  0, 0,  1];
    else
    T_HFEattachment = [0,  0, -1, 0;
                       0,  1, 0, -l_hip;
                       1,  0, 0,  0;
                       0,  0, 0,  1];
end

% HFE to KFE translation
T_HFE =           [1, 0, 0, l_thigh;
                   0  1, 0, 0;
                   0, 0, 1, 0;
                   0, 0, 0, 1];

% KFE to EE (or KFE to AFE for higher link counts) translation
T_KFE =           [1, 0, 0, l_shank;
                   0, 1, 0, 0;
                   0, 0, 1, 0;
                   0, 0, 0, 1];

if (linkCount == 3) | (linkCount ==4)
    % AFE to EE (or AFE to DFE for higher link counts) translation
    T_AFE =           [1, 0, 0, l_foot;
                       0, 1, 0, 0;
                       0, 0, 1, 0;
                       0, 0, 0, 1];
end
if (linkCount ==4)
    % DFE to EE translation
    T_DFE =           [1, 0, 0, l_phalanges;
                       0, 1, 0, 0;
                       0, 0, 1, 0;
                       0, 0, 0, 1];
end
%% Create and assemble rigid bodies
% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree('DataFormat', 'row');
% Create bodies and joints 
body0 = robotics.RigidBody('body0');
body1 = robotics.RigidBody('body1'); % hip
body2 = robotics.RigidBody('body2'); % thigh
body3 = robotics.RigidBody('body3'); % shank
body4 = robotics.RigidBody('body4'); % EE or foot

if (linkCount == 3) | (linkCount == 4)
       body5 = robotics.RigidBody('body5'); % EE or phalanges
end
if (linkCount == 4)
       body6 = robotics.RigidBody('body6'); % EE
end

jnt0 = robotics.Joint('jnt0','revolute'); % body rotation about y in inertial frame
jnt1 = robotics.Joint('jnt1','revolute'); % HAA
jnt2 = robotics.Joint('jnt2','revolute'); % HFE
jnt3 = robotics.Joint('jnt3','revolute'); % KFE
jnt4 = robotics.Joint('jnt4','fixed'); % coordinate system at EE 
if (linkCount == 3)
       jnt4 = robotics.Joint('jnt4','revolute'); % AFE
       jnt5 = robotics.Joint('jnt5','fixed'); % EE
end
if (linkCount == 4)
       jnt4 = robotics.Joint('jnt4','revolute'); % AFE
       jnt5 = robotics.Joint('jnt5','revolute'); % DFE
       jnt6 = robotics.Joint('jnt6','fixed'); % EE
end

body0.Mass = 0;      
body1.Mass = quadruped.hip(selectFrontHind).mass;    % hip  
body2.Mass = quadruped.thigh(selectFrontHind).mass;  % thigh
body3.Mass = quadruped.shank(selectFrontHind).mass;  % shank
body4.Mass = quadruped.EE(selectFrontHind).mass;     % EE
if (linkCount == 3)
    body4.Mass = quadruped.foot(selectFrontHind).mass;  % overwrite EE with foot
    body5.Mass = quadruped.EE(selectFrontHind).mass;  % EE
end
if (linkCount == 4)
    body4.Mass = quadruped.foot(selectFrontHind).mass;  % foot
    body5.Mass = quadruped.phalanges(selectFrontHind).mass;  % overwrite EE with phalanges
    body6.Mass = quadruped.EE(selectFrontHind).mass;  % EE
end

% inertia = [Ixx Iyy Izz Iyz Ixz Ixy] relative to body frame in kg/m^2
% note that body3.Inertia encompasses shank and end effector inertia terms
body0.Inertia = [0 0 0 0 0 0];      
body1.Inertia = [0.00000001  1/3*body1.Mass*l_hip^2                             1/3*body1.Mass*l_hip^2                             0 0 0]; %hip      
body2.Inertia = [0.00000001  1/3*body2.Mass*l_thigh^2                           1/3*body2.Mass*l_thigh^2                           0 0 0]; %thigh
body3.Inertia = [0.00000001  1/3*body3.Mass*l_shank^2+1/2*body4.Mass*l_shank^2  1/3*body3.Mass*l_shank^2+1/2*body4.Mass*l_shank^2  0 0 0]; %shank and EE inertia
body4.Inertia = [0 0 0 0 0 0];

if (linkCount == 3)
    body3.Inertia = [0.00000001  1/3*body3.Mass*l_shank^2  1/3*body3.Mass*l_shank^2  0 0 0]; %shank without EE
    body4.Inertia = [0.00000001  1/3*body4.Mass*l_foot^2+1/2*body5.Mass*l_foot^2  1/3*body4.Mass*l_foot^2+1/2*body5.Mass*l_foot^2  0 0 0]; % foot with EE
end

if (linkCount == 4)
    body4.Inertia = [0.00000001  1/3*body4.Mass*l_foot^2  1/3*body4.Mass*l_foot^2  0 0 0]; %foot without EE
    body5.Inertia = [0.00000001  1/3*body5.Mass*l_phalanges^2+1/2*body6.Mass*l_phalanges^2  1/3*body5.Mass*l_phalanges^2+1/2*body6.Mass*l_phalanges^2  0 0 0]; % phalanges with EE
end

% center of mass and mass terms do not affect inertia but are used 
% to compute torque due to gravitational force
body1.CenterOfMass = [0.5*quadruped.hip(selectFrontHind).length   0 0];
body2.CenterOfMass = [0.5*quadruped.thigh(selectFrontHind).length 0 0];
body3.CenterOfMass = [0.5*quadruped.shank(selectFrontHind).length 0 0]; 
body4.CenterOfMass = [0 0 0];

% set joint transforms - these are only translations and rotations to align rotation
% z with joint rotation axis. The joint positions are specified in the config array.   
setFixedTransform(jnt0, T_body);
setFixedTransform(jnt1, T_HAA);
setFixedTransform(jnt2, T_HFEattachment);
setFixedTransform(jnt3, T_HFE);
setFixedTransform(jnt4, T_KFE); 
if (linkCount == 3) | (linkCount == 4)
    setFixedTransform(jnt5, T_AFE);    
end
if (linkCount == 4)
    setFixedTransform(jnt6, T_DFE);            
end

body0.Joint = jnt0;
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
if (linkCount == 3) || (linkCount == 4)
    body5.Joint = jnt5;
end
if (linkCount == 4)
    body6.Joint = jnt6;
end

%% specify connections between bodies
addBody(robot, body0,'base');
addBody(robot, body1,'body0');
addBody(robot, body2,'body1');
addBody(robot, body3,'body2');
addBody(robot, body4,'body3');
if (linkCount == 3) | (linkCount == 4)
    addBody(robot, body5,'body4');
end
if (linkCount == 4)
    addBody(robot, body6,'body5');
end

robot.Gravity = [0 0 -9.8];
            
%% Display robot 
for i = 1:length(Leg.(EEselection).q)
    if (linkCount == 2)
        config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles(i,2), ... %body rotation about inertial y
                       Leg.(EEselection).q(i,1), ... % HAA
                       Leg.(EEselection).q(i,2), ... % HFE
                       Leg.(EEselection).q(i,3)]; % KFE
    end
    
    if (linkCount == 3)
        config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles(i,2), ... %body rotation about inertial y
                       Leg.(EEselection).q(i,1), ... % HAA
                       Leg.(EEselection).q(i,2), ... % HFE
                       Leg.(EEselection).q(i,3), ... % KFE
                       Leg.(EEselection).q(i,4)];    % AFE
    end
    
    if (linkCount == 4)
        config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles(i,2), ... %body rotation about inertial y
                       Leg.(EEselection).q(i,1), ... % HAA
                       Leg.(EEselection).q(i,2), ... % HFE
                       Leg.(EEselection).q(i,3), ... % KFE
                       Leg.(EEselection).q(i,4), ... % AFE
                       Leg.(EEselection).q(i,5)];    % DFE
    end
end

figure(1);

if viewVisualization
    for j = 1: numberOfLoopRepetitions
        for i = 1:length(Leg.(EEselection).q)
            set(gcf, 'Position', get(0, 'Screensize'));
            xlim([-0.7 0.7]);
            ylim([-0.1 0.1]);
            zlim([-0.8 0.2]);
            figure(1);
            show(robot,config(i,:));
            hold on
            % plot desired trajectory to observe tracking
            plot3(meanCyclicMotionHipEE.(EEselection).position(:,1), ...
                  meanCyclicMotionHipEE.(EEselection).position(:,2), ...
                  meanCyclicMotionHipEE.(EEselection).position(:,3),'r', 'LineWidth', 3)
            title(EEselection)
            hold off
        end
    end
end

