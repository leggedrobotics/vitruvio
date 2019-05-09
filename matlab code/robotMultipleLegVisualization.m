%% Read in data for quadruped geometry
% function robot = robotMultipleLegVisualization(quadruped, q, meanCyclicC_IBody, EE, meanCyclicMotionHipEE, EESelection, reachablePositions, numberOfLoopRepetitions) 

%% get quadruped properties for selected end effector

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
EE_name = fieldnames(EE);

%% Build quadruped rigid body model using 4x4 transformation matrices

meanCyclicC_IBody(4,4,:) = 1;

            

for k = 1:1 % repeat visualization loop
    for i = 1:length(q.LF);
        for j = 1:4 % displays all legs
            
            % transformation from HAA to hip attachment
            % rotation about z of hip attachment
            
            
            if ( j < 3)
                selectFrontHind = 1;
            else SelectFrontHind = 2;
            end

l_hip = quadruped.hip(selectFrontHind).length;
l_thigh = quadruped.thigh(selectFrontHind).length;
l_shank = quadruped.shank(selectFrontHind).length;

            
%             need to change something in IK for this formulation to work
%             T_HB.(EE_name{j})(:,:,i) = meanCyclicC_IBody(:,:,i)
            T_HB.(EE_name{j}) = [1 0 0 xNom.(EE_name{j});
                                 0 1 0 yNom.(EE_name{j});
                                 0 0 1 zNom;
                                 0 0 0 1];
             
            
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
            
             
            %% Create and assemble rigid bodies
            
            % Create a rigid body tree object to build the robot.
            robot = robotics.RigidBodyTree('DataFormat', 'row');
            
            body1 = robotics.RigidBody('body1');
            jnt1 = robotics.Joint('jnt1','fixed');
            
            setFixedTransform(jnt1,eye(4));
            body1.Joint = jnt1;
            
            addBody(robot,body1,'base');
            
            %% Define bodies and joints
            
            % LF leg
            body2 = robotics.RigidBody('body2');
            body3 = robotics.RigidBody('body3');
            body4 = robotics.RigidBody('body4');
            body5 = robotics.RigidBody('body5');
            body6 = robotics.RigidBody('body6');
            
            jnt2 = robotics.Joint('jnt2','revolute'); % Hip attachment point
            jnt3 = robotics.Joint('jnt3','revolute'); % HAA
            jnt4 = robotics.Joint('jnt4','revolute'); % HFE
            jnt5 = robotics.Joint('jnt5','revolute'); % KFE
            jnt6 = robotics.Joint('jnt6','revolute'); % EE
            
            body2.Mass = 0;
            body3.Mass = quadruped.hip(selectFrontHind).mass;
            body4.Mass = quadruped.thigh(selectFrontHind).mass;
            body5.Mass = quadruped.shank(selectFrontHind).mass;
            body6.Mass = quadruped.shank(selectFrontHind).mass;
            
            
            body3.CenterOfMass = [0 0 -0.5*quadruped.hip(selectFrontHind).length];
            body4.CenterOfMass = [0 0 -0.5*quadruped.thigh(selectFrontHind).length];
            body5.CenterOfMass = [0 0 -0.5*quadruped.shank(selectFrontHind).length];
            
            % LH leg
            body7 = robotics.RigidBody('body7');
            jnt7 = robotics.Joint('jnt7','fixed'); % hip attachment
            body8 = robotics.RigidBody('body8');
            jnt8 = robotics.Joint('jnt8','revolute'); % HAA
            body9 = robotics.RigidBody('body9');
            jnt9 = robotics.Joint('jnt9','revolute'); % HFE
            body10 = robotics.RigidBody('body10');
            jnt10 = robotics.Joint('jnt10','revolute'); % KFE
            body11 = robotics.RigidBody('body11');
            jnt11 = robotics.Joint('jnt11','revolute'); % EE
           
            % RF leg
            body12 = robotics.RigidBody('body12');
            jnt12 = robotics.Joint('jnt12','fixed'); % hip attachment
            body13 = robotics.RigidBody('body13');
            jnt13 = robotics.Joint('jnt13','revolute'); % HAA
            body14 = robotics.RigidBody('body14');
            jnt14 = robotics.Joint('jnt14','revolute'); % HFE
            body15 = robotics.RigidBody('body15');
            jnt15 = robotics.Joint('jnt15','revolute'); % KFE
            body16 = robotics.RigidBody('body16');
            jnt16 = robotics.Joint('jnt16','revolute'); % EE
            
            % RH leg
            body17 = robotics.RigidBody('body17');
            jnt17 = robotics.Joint('jnt17','fixed'); % hip attachment
            body18 = robotics.RigidBody('body18');
            jnt18 = robotics.Joint('jnt18','revolute'); % HAA
            body19 = robotics.RigidBody('body19');
            jnt19 = robotics.Joint('jnt19','revolute'); % HFE
            body20 = robotics.RigidBody('body20');
            jnt20 = robotics.Joint('jnt20','revolute'); % KFE
            body21 = robotics.RigidBody('body21');
            jnt21 = robotics.Joint('jnt21','revolute'); % EE

            
            % addVisual(body3,"Mesh",'thigh2_universal.stl')
            % addVisual(body4, "Mesh",'shank2_universal.stl')
            
            
            %% set joint transforms
            setFixedTransform(jnt2, T_HB.(EE_name{j}));
            setFixedTransform(jnt3, T_H1.(EE_name{j})(:,:,i));
            setFixedTransform(jnt4,  T_12.(EE_name{j})(:,:,i));
            setFixedTransform(jnt5, T_23.(EE_name{j})(:,:,i));
            setFixedTransform(jnt6, T_34.(EE_name{j})(:,:,i));
            
            setFixedTransform(jnt7, T_HB.(EE_name{j}));
            setFixedTransform(jnt8, T_H1.(EE_name{j})(:,:,i));
            setFixedTransform(jnt9, T_12.(EE_name{j})(:,:,i));
            setFixedTransform(jnt10, T_23.(EE_name{j})(:,:,i));
            setFixedTransform(jnt11, T_34.(EE_name{j})(:,:,i));

            setFixedTransform(jnt12, T_HB.(EE_name{j}));
            setFixedTransform(jnt13, T_H1.(EE_name{j})(:,:,i));
            setFixedTransform(jnt14, T_12.(EE_name{j})(:,:,i));
            setFixedTransform(jnt15, T_23.(EE_name{j})(:,:,i));
            setFixedTransform(jnt16, T_34.(EE_name{j})(:,:,i));

            setFixedTransform(jnt17, T_HB.(EE_name{j}));
            setFixedTransform(jnt18, T_H1.(EE_name{j})(:,:,i));
            setFixedTransform(jnt19, T_12.(EE_name{j})(:,:,i));
            setFixedTransform(jnt20, T_23.(EE_name{j})(:,:,i));
            setFixedTransform(jnt21, T_34.(EE_name{j})(:,:,i));
            
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
            body18.Joint = jnt18;
            body19.Joint = jnt19;
            body20.Joint = jnt20;
            body21.Joint = jnt21;
                        
            %% specify connections between bodies
            addBody(robot,body2,'body1');
            addBody(robot,body3,'body2');
            addBody(robot,body4,'body3');
            addBody(robot,body5,'body4');
            addBody(robot,body6,'body5');
            
            addBody(robot,body7,'body1')
            addBody(robot,body8,'body7')
            addBody(robot,body9,'body8')
            addBody(robot,body10,'body9')
            addBody(robot,body11,'body10')
             
            addBody(robot,body12,'body1')
            addBody(robot,body13,'body12')
            addBody(robot,body14,'body13')
            addBody(robot,body15,'body14')
            addBody(robot,body16,'body15')

            addBody(robot,body17,'body1')
            addBody(robot,body18,'body17')
            addBody(robot,body19,'body18')
            addBody(robot,body20,'body19')
            addBody(robot,body21,'body20')
            
            %% Change joint axis for hip and knee extension to y axis and hip flexion to x axis
            %  Does this even have an effect?
            jnt2.JointAxis = [1 0 0];
            jnt3.JointAxis = [0 1 0];
            jnt4.JointAxis = [0 1 0];
            jnt5.JointAxis = [0 1 0];

            jnt8.JointAxis = [1 0 0];
            jnt9.JointAxis = [0 1 0];
            jnt10.JointAxis = [0 1 0];
             
            jnt13.JointAxis = [1 0 0];
            jnt14.JointAxis = [0 1 0];
            jnt15.JointAxis = [0 1 0];

            jnt18.JointAxis = [1 0 0];
            jnt19.JointAxis = [0 1 0];
            jnt20.JointAxis = [0 1 0];
                        
            %% Display robot and details
        end
        
            figure(10)

            robot.Gravity = [0 0 -9.8];
            show(robot); %, 'Visuals', "on");
                        
            hold on
            plot3(meanCyclicMotionHipEE.(EE_name{j}).position(:,1),meanCyclicMotionHipEE.(EE_name{j}).position(:,2),meanCyclicMotionHipEE.(EE_name{j}).position(:,3),'r', 'LineWidth', 3)
            hold off
            
%             xlim([-0.5 0.5]);
%             ylim([-0.5 0.5]);
%             zlim([-0.8 0.2]);
            
        end
    end


%% might include other legs in a visualization in the future


% 
% % %% LH leg
% body7 = robotics.RigidBody('body7');
% jnt7 = robotics.Joint('jnt7','fixed'); % hip attachment
% body8 = robotics.RigidBody('body8');
% jnt8 = robotics.Joint('jnt8','revolute'); % HAA
% body9 = robotics.RigidBody('body9');
% jnt9 = robotics.Joint('jnt9','revolute'); % HFE
% body10 = robotics.RigidBody('body10');
% jnt10 = robotics.Joint('jnt10','revolute'); % KFE
% body11 = robotics.RigidBody('body11');
% jnt11 = robotics.Joint('jnt11','revolute'); % EE
% %  
% % %% RF leg
% body12 = robotics.RigidBody('body12');
% jnt12 = robotics.Joint('jnt12','fixed'); % hip attachment
% body13 = robotics.RigidBody('body13');
% jnt13 = robotics.Joint('jnt13','revolute'); % HAA
% body14 = robotics.RigidBody('body14');
% jnt14 = robotics.Joint('jnt14','revolute'); % HFE
% body15 = robotics.RigidBody('body15');
% jnt15 = robotics.Joint('jnt15','revolute'); % KFE
% body16 = robotics.RigidBody('body16');
% jnt16 = robotics.Joint('jnt16','revolute'); % EE
% % 
% %  
% % %% RH leg
% body17 = robotics.RigidBody('body17');
% jnt17 = robotics.Joint('jnt17','fixed'); % hip attachment
% body18 = robotics.RigidBody('body18');
% jnt18 = robotics.Joint('jnt18','revolute'); % HAA
% body19 = robotics.RigidBody('body19');
% jnt19 = robotics.Joint('jnt19','revolute'); % HFE
% body20 = robotics.RigidBody('body20');
% jnt20 = robotics.Joint('jnt20','revolute'); % KFE
% body21 = robotics.RigidBody('body21');
% jnt21 = robotics.Joint('jnt21','revolute'); % EE


% setFixedTransform(jnt7, T_HB.LH);
% setFixedTransform(jnt8, T_H1.LH(:,:,i));
% setFixedTransform(jnt9, T_12.LH(:,:,i));
% setFixedTransform(jnt10, T_23.LH(:,:,i));
% setFixedTransform(jnt11, T_34.LH(:,:,i));
% 
% setFixedTransform(jnt12, T_HB.RF);
% setFixedTransform(jnt13, T_H1.RF(:,:,i));
% setFixedTransform(jnt14, T_12.RF(:,:,i));
% setFixedTransform(jnt15, T_23.RF(:,:,i));
% setFixedTransform(jnt16, T_34.RF(:,:,i));
% 
% setFixedTransform(jnt17, T_HB.RH);
% setFixedTransform(jnt18, T_H1.RH(:,:,i));
% setFixedTransform(jnt19, T_12.RH(:,:,i));
% setFixedTransform(jnt20, T_23.RH(:,:,i));
% setFixedTransform(jnt21, T_34.RH(:,:,i));

% 
% body7.Joint = jnt7;
% body8.Joint = jnt8;
% body9.Joint = jnt9;
% body10.Joint = jnt10;
% body11.Joint = jnt11;
% % 
% body12.Joint = jnt12;
% body13.Joint = jnt13;
% body14.Joint = jnt14;
% body15.Joint = jnt15;
% body16.Joint = jnt16;
% 
% body17.Joint = jnt17;
% body18.Joint = jnt18;
% body19.Joint = jnt19;
% body20.Joint = jnt20;
% body21.Joint = jnt21;

% 
% addBody(robot,body7,'body1')
% addBody(robot,body8,'body7')
% addBody(robot,body9,'body8')
% addBody(robot,body10,'body9')
% addBody(robot,body11,'body10')
% % 
% addBody(robot,body12,'body1')
% addBody(robot,body13,'body12')
% addBody(robot,body14,'body13')
% addBody(robot,body15,'body14')
% addBody(robot,body16,'body15')
% 
% addBody(robot,body17,'body1')
% addBody(robot,body18,'body17')
% addBody(robot,body19,'body18')
% addBody(robot,body20,'body19')
% addBody(robot,body21,'body20')



% jnt8.JointAxis = [1 0 0];
% jnt9.JointAxis = [0 1 0];
% jnt10.JointAxis = [0 1 0];
% % 
% jnt13.JointAxis = [1 0 0];
% jnt14.JointAxis = [0 1 0];
% jnt15.JointAxis = [0 1 0];
% 
% jnt18.JointAxis = [1 0 0];
% jnt19.JointAxis = [0 1 0];
% jnt20.JointAxis = [0 1 0];
