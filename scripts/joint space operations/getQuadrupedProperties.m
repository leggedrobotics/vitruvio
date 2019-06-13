%% Nominal robot classes
% the link lengths need to be updated during the optimization so this data
% cannot be saved into a .mat file like the motion data

function quadruped = getQuadrupedProperties(robotSelection, linkCount)
legDensity = 249.4; %kg based on desired total leg mass for universal. Could be updated for other robots.

%% ANYmal
robot.ANYmal.mass.total = 29.5; 

% offset from CoM to each hip
robot.ANYmal.xNom(1) = 0.34;
robot.ANYmal.xNom(2) = 0.34;
robot.ANYmal.yNom(1) = 0.19;
robot.ANYmal.yNom(2) = 0.19;
robot.ANYmal.zNom = 0.05; % offset from CoM to hip attachment in z direction Hip height 0.47m CoM at 0.42

% row order:    LF LH RF RH
% column order: x, y, z
robot.ANYmal.nomHipPos(1,:) = [ robot.ANYmal.xNom(1),  robot.ANYmal.yNom(1), robot.ANYmal.zNom];
robot.ANYmal.nomHipPos(2,:) = [-robot.ANYmal.xNom(2),  robot.ANYmal.yNom(2), robot.ANYmal.zNom];
robot.ANYmal.nomHipPos(3,:) = [ robot.ANYmal.xNom(1), -robot.ANYmal.yNom(1), robot.ANYmal.zNom];
robot.ANYmal.nomHipPos(4,:) = [-robot.ANYmal.xNom(2), -robot.ANYmal.yNom(2), robot.ANYmal.zNom];

% link lengths [m]
% fore, hind
robot.ANYmal.hip(1).length = 0.14;
robot.ANYmal.hip(2).length = 0.14;
robot.ANYmal.thigh(1).length = 0.25;
robot.ANYmal.thigh(2).length = 0.25;
robot.ANYmal.shank(1).length = 0.33;
robot.ANYmal.shank(2).length = 0.33;
robot.ANYmal.foot(1).length = 0.05;
robot.ANYmal.foot(2).length = 0.05;
robot.ANYmal.phalanges(1).length = 0.1;
robot.ANYmal.phalanges(2).length = 0.1;

% link radius [m]
% update these values
robot.ANYmal.hip(1).radius = 0.05;
robot.ANYmal.hip(2).radius = 0.05;
robot.ANYmal.thigh(1).radius = 0.05;
robot.ANYmal.thigh(2).radius = 0.05;
robot.ANYmal.shank(1).radius = 0.05;
robot.ANYmal.shank(2).radius = 0.05;
robot.ANYmal.foot(1).radius = 0.05;
robot.ANYmal.foot(2).radius = 0.05;
robot.ANYmal.phalanges(1).radius = 0.05;
robot.ANYmal.phalanges(2).radius = 0.05;

robot.ANYmal.legDensity = legDensity; %kg/m^3

%link mass [kg] and inertia [kg.m^2] based on cylindrical link with constant density
link = {'hip','thigh' 'shank' 'foot' 'phalanges'};
for i = 1:length(link)
    for j = 1:2
        robot.ANYmal.(link{i})(j).mass = pi()*robot.ANYmal.(link{i})(j).radius^2*robot.ANYmal.(link{i})(j).length*robot.ANYmal.legDensity;
        robot.ANYmal.(link{i})(j).inertia = (1/3)*robot.ANYmal.(link{i})(j).mass*robot.ANYmal.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.ANYmal.q1.minAngle = -pi/2;
robot.ANYmal.q1.maxAngle = pi/2;
robot.ANYmal.q2.minAngle = -pi/2;
robot.ANYmal.q2.maxAngle = pi/2;
robot.ANYmal.q3.minAngle = -pi/2;
robot.ANYmal.q3.maxAngle = pi/2;
robot.ANYmal.q4.minAngle = -pi/2;
robot.ANYmal.q4.maxAngle = pi/2;

%% Universal
robot.universal.mass.total = 39.53; % with payload

% offset from CoM to each hip
robot.universal.xNom(1) = 0.34;
robot.universal.xNom(2) = 0.34;
robot.universal.yNom(1) = 0.19;
robot.universal.yNom(2) = 0.19;
robot.universal.zNom = -0.566+0.4695; % offset from CoM to hip attachment

% row order:    LF LH RF RH
% column order: x, y, z
robot.universal.nomHipPos(1,:) = [robot.universal.xNom(1), robot.universal.yNom(1), robot.universal.zNom];
robot.universal.nomHipPos(2,:) = [-robot.universal.xNom(2), robot.universal.yNom(2), robot.universal.zNom];
robot.universal.nomHipPos(3,:) = [robot.universal.xNom(1), -robot.universal.yNom(1), robot.universal.zNom];
robot.universal.nomHipPos(4,:) = [-robot.universal.xNom(2), -robot.universal.yNom(2), robot.universal.zNom];

% link lengths [m]
% fore, hind
robot.universal.hip(1).length = 0.14;
robot.universal.hip(2).length = 0.14;
robot.universal.thigh(1).length = 0.3;
robot.universal.thigh(2).length = 0.3;
robot.universal.shank(1).length = 0.3;
robot.universal.shank(2).length = 0.3;
robot.universal.foot(1).length = 0.1;
robot.universal.foot(2).length = 0.1;
robot.universal.phalanges(1).length = 0.05;
robot.universal.phalanges(2).length = 0.1;

% link radius [m]
% update these values
robot.universal.hip(1).radius = 0.05;
robot.universal.hip(2).radius = 0.05;
robot.universal.thigh(1).radius = 0.05;
robot.universal.thigh(2).radius = 0.05;
robot.universal.shank(1).radius = 0.05;
robot.universal.shank(2).radius = 0.05;
robot.universal.foot(1).radius = 0.05;
robot.universal.foot(2).radius = 0.05;
robot.universal.phalanges(1).radius = 0.05;
robot.universal.phalanges(2).radius = 0.05;

robot.universal.legDensity = legDensity; %kg/m^3

%link mass [kg] based on cylindrical link with constant density
for i = 1:length(link)
    for j = 1:2
        robot.universal.(link{i})(j).mass = pi()*robot.universal.(link{i})(j).radius^2*robot.universal.(link{i})(j).length*robot.universal.legDensity;
        robot.universal.(link{i})(j).inertia = (1/3)*robot.universal.(link{i})(j).mass*robot.universal.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.universal.q1.minAngle = -pi/2;
robot.universal.q1.maxAngle = pi/2;
robot.universal.q2.minAngle = -pi/2;
robot.universal.q2.maxAngle = pi/2;
robot.universal.q3.minAngle = -pi/2;
robot.universal.q3.maxAngle = pi/2;
robot.universal.q4.minAngle = -pi/2;
robot.universal.q4.maxAngle = pi/2;

%% Speedy 

robot.speedy.mass.total = 22.52;

% offset from CoM to each hip
robot.speedy.xNom(1) = 0.31;
robot.speedy.xNom(2) = 0.31;
robot.speedy.yNom(1) = 0.1; % front
robot.speedy.yNom(2) = 0.14; % hind
robot.speedy.zNom = -0.304 + 0.47;

% row order:    LF LH RF RH
% column order: x, y, z
robot.speedy.nomHipPos(1,:) = [robot.speedy.xNom(1), robot.speedy.yNom(1), robot.speedy.zNom];
robot.speedy.nomHipPos(2,:) = [-robot.speedy.xNom(2), robot.speedy.yNom(2), robot.speedy.zNom];
robot.speedy.nomHipPos(3,:) = [robot.speedy.xNom(1), -robot.speedy.yNom(1), robot.speedy.zNom];
robot.speedy.nomHipPos(4,:) = [-robot.speedy.xNom(2), -robot.speedy.yNom(2), robot.speedy.zNom];

% link lengths [m]
% fore, hind
robot.speedy.hip(1).length = 0.15;
robot.speedy.hip(2).length = 0.15;
robot.speedy.thigh(1).length = 0.5;
robot.speedy.thigh(2).length = 0.5;
robot.speedy.shank(1).length = 0.45;
robot.speedy.shank(2).length = 0.45;
robot.speedy.foot(1).length = 0.15;
robot.speedy.foot(2).length = 0.15;
robot.speedy.phalanges(1).length = 0.1;
robot.speedy.phalanges(2).length = 0.1;


% link radius [m]
robot.speedy.hip(1).radius = 0.02;
robot.speedy.hip(2).radius = 0.02;
robot.speedy.thigh(1).radius = 0.02;
robot.speedy.thigh(2).radius = 0.03;
robot.speedy.shank(1).radius = 0.03;
robot.speedy.shank(2).radius = 0.03;
robot.speedy.foot(1).radius = 0.03;
robot.speedy.foot(2).radius = 0.03;
robot.speedy.phalanges(1).radius = 0.03;
robot.speedy.phalanges(2).radius = 0.03;

robot.speedy.legDensity = legDensity; %kg/m^3

%link mass [kg] based on cylindrical link with constant density
for i = 1:length(link)
    for j = 1:2
        robot.speedy.(link{i})(j).mass = pi()*robot.speedy.(link{i})(j).radius^2*robot.speedy.(link{i})(j).length*robot.speedy.legDensity;
        robot.speedy.(link{i})(j).inertia = (1/3)*robot.speedy.(link{i})(j).mass*robot.speedy.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.speedy.q1.minAngle = -pi/6;
robot.speedy.q1.maxAngle = pi/2;
robot.speedy.q2.minAngle = -pi/4;
robot.speedy.q2.maxAngle = pi/4;
robot.speedy.q3.minAngle = -pi/2;
robot.speedy.q3.maxAngle = pi/2;
robot.speedy.q4.minAngle = pi/2;
robot.speedy.q4.maxAngle = -pi/2;

%% Massivo 
robot.massivo.mass.total = 80;

% offset from CoM to each hip
robot.massivo.xNom(1) = 0.276;
robot.massivo.xNom(2) = 0.276;
robot.massivo.yNom(1) = 0.3;
robot.massivo.yNom(2) = 0.3;
robot.massivo.zNom = -0.553 + 0.5043;

% row order:    LF LH RF RH
% column order: x, y, z
robot.massivo.nomHipPos(1,:) = [robot.massivo.xNom(1), robot.massivo.yNom(1), robot.massivo.zNom];
robot.massivo.nomHipPos(2,:) = [-robot.massivo.xNom(2), robot.massivo.yNom(2), robot.massivo.zNom];
robot.massivo.nomHipPos(3,:) = [robot.massivo.xNom(1), -robot.massivo.yNom(1), robot.massivo.zNom];
robot.massivo.nomHipPos(4,:) = [-robot.massivo.xNom(2), -robot.massivo.yNom(2), robot.massivo.zNom];

% link lengths [m]
% fore, hind
robot.massivo.hip(1).length = 0.15;
robot.massivo.hip(2).length = 0.15;
robot.massivo.thigh(1).length = 0.5;
robot.massivo.thigh(2).length = 0.5;
robot.massivo.shank(1).length = 0.5;
robot.massivo.shank(2).length = 0.5;
robot.massivo.foot(1).length = 0.1;
robot.massivo.foot(2).length = 0.1;
robot.massivo.phalanges(1).length = 0.05;
robot.massivo.phalanges(2).length = 0.05;


% link radius [m]
robot.massivo.hip(1).radius = 0.02;
robot.massivo.hip(2).radius = 0.02;
robot.massivo.thigh(1).radius = 0.02;
robot.massivo.thigh(2).radius = 0.03;
robot.massivo.shank(1).radius = 0.03;
robot.massivo.shank(2).radius = 0.03;
robot.massivo.foot(1).radius = 0.03;
robot.massivo.foot(2).radius = 0.03;
robot.massivo.phalanges(1).radius = 0.03;
robot.massivo.phalanges(2).radius = 0.03;

robot.massivo.legDensity = legDensity; %kg/m^3

%link mass [kg] based on cylindrical link with constant density
for i = 1:length(link)
    for j = 1:2
        robot.massivo.(link{i})(j).mass = pi()*robot.massivo.(link{i})(j).radius^2*robot.massivo.(link{i})(j).length*robot.massivo.legDensity;
        robot.massivo.(link{i})(j).inertia = (1/3)*robot.massivo.(link{i})(j).mass*robot.massivo.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.massivo.q1.minAngle = -pi/6;
robot.massivo.q1.maxAngle = pi/2;
robot.massivo.q2.minAngle = -pi/4;
robot.massivo.q2.maxAngle = pi/4;
robot.massivo.q3.minAngle = pi/2;
robot.massivo.q3.maxAngle = -pi/2;
robot.massivo.q4.minAngle = pi/2;
robot.massivo.q4.maxAngle = -pi/2;

%% Centaur 
robot.centaur.mass.total = 80;

% offset from CoM to each hip
robot.centaur.xNom(1) = 0.1451;
robot.centaur.xNom(2) = 0.407;
robot.centaur.yNom(1) = 0.3;
robot.centaur.yNom(2) = 0.3;
robot.centaur.zNom = -0.553 + 0.5043;

% row order:    LF LH RF RH
% column order: x, y, z
robot.centaur.nomHipPos(1,:) = [robot.centaur.xNom(1), robot.centaur.yNom(1), robot.centaur.zNom];
robot.centaur.nomHipPos(2,:) = [-robot.centaur.xNom(2), robot.centaur.yNom(2), robot.centaur.zNom];
robot.centaur.nomHipPos(3,:) = [robot.centaur.xNom(1), -robot.centaur.yNom(1), robot.centaur.zNom];
robot.centaur.nomHipPos(4,:) = [-robot.centaur.xNom(2), -robot.centaur.yNom(2), robot.centaur.zNom];

% link lengths [m]
% fore, hind
robot.centaur.hip(1).length = 0.15;
robot.centaur.hip(2).length = 0.15;
robot.centaur.thigh(1).length = 0.5;
robot.centaur.thigh(2).length = 0.5;
robot.centaur.shank(1).length = 0.5;
robot.centaur.shank(2).length = 0.5;
robot.centaur.foot(1).length = 0.05;
robot.centaur.foot(2).length = 0.05;
robot.centaur.phalanges(1).length = 0.05;
robot.centaur.phalanges(2).length = 0.05;

% link radius [m]
robot.centaur.hip(1).radius = 0.02;
robot.centaur.hip(2).radius = 0.02;
robot.centaur.thigh(1).radius = 0.02;
robot.centaur.thigh(2).radius = 0.03;
robot.centaur.shank(1).radius = 0.03;
robot.centaur.shank(2).radius = 0.03;
robot.centaur.foot(1).radius = 0.03;
robot.centaur.foot(2).radius = 0.03;
robot.centaur.phalanges(1).radius = 0.03;
robot.centaur.phalanges(2).radius = 0.03;

robot.centaur.legDensity = legDensity; %kg/m^3

%link mass [kg] based on cylindrical link with constant density
for i = 1:length(link)
    for j = 1:2
        robot.centaur.(link{i})(j).mass = pi()*robot.centaur.(link{i})(j).radius^2*robot.centaur.(link{i})(j).length*robot.centaur.legDensity;
        robot.centaur.(link{i})(j).inertia = (1/3)*robot.centaur.(link{i})(j).mass*robot.centaur.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.centaur.q1.minAngle = -pi/6;
robot.centaur.q1.maxAngle = pi/2;
robot.centaur.q2.minAngle = -pi/4;
robot.centaur.q2.maxAngle = pi/4;
robot.centaur.q3.minAngle = pi/2;
robot.centaur.q3.maxAngle = -pi/2;
robot.centaur.q4.minAngle = pi/2;
robot.centaur.q4.maxAngle = -pi/2;

%% Mini 

robot.mini.mass.total = 10;

robot.mini.xNom(1) = 0.18;
robot.mini.xNom(2) = 0.18;
robot.mini.yNom(1) = 0.1;
robot.mini.yNom(2) = 0.1;
robot.mini.zNom = -0.186 + 0.198;

% row order:    LF LH RF RH
% column order: x, y, z
robot.mini.nomHipPos(1,:) = [robot.mini.xNom(1), robot.mini.yNom(1), robot.mini.zNom];
robot.mini.nomHipPos(2,:) = [-robot.mini.xNom(2), robot.mini.yNom(2), robot.mini.zNom];
robot.mini.nomHipPos(3,:) = [robot.mini.xNom(1), -robot.mini.yNom(1), robot.mini.zNom];
robot.mini.nomHipPos(4,:) = [-robot.mini.xNom(2), -robot.mini.yNom(2), robot.mini.zNom];

% link lengths [m]
% fore, hind
robot.mini.hip(1).length = 0.05;
robot.mini.hip(2).length = 0.05;
robot.mini.thigh(1).length = 0.14;
robot.mini.thigh(2).length = 0.14;
robot.mini.shank(1).length = 0.14;
robot.mini.shank(2).length = 0.14;
robot.mini.foot(1).length = 0.05;
robot.mini.foot(2).length = 0.05;
robot.mini.phalanges(1).length = 0.05;
robot.mini.phalanges(2).length = 0.05;

% link radius [m]
robot.mini.hip(1).radius = 0.02;
robot.mini.hip(2).radius = 0.02;
robot.mini.thigh(1).radius = 0.02;
robot.mini.thigh(2).radius = 0.03;
robot.mini.shank(1).radius = 0.03;
robot.mini.shank(2).radius = 0.03;
robot.mini.foot(1).radius = 0.03;
robot.mini.foot(2).radius = 0.03;
robot.mini.phalanges(1).radius = 0.03;
robot.mini.phalanges(2).radius = 0.03;

robot.mini.legDensity = legDensity; %kg/m^3

%link mass [kg] based on cylindrical link with constant density
for i = 1:length(link)
    for j = 1:2
        robot.mini.(link{i})(j).mass = pi()*robot.mini.(link{i})(j).radius^2*robot.mini.(link{i})(j).length*robot.mini.legDensity;
        robot.mini.(link{i})(j).inertia = (1/3)*robot.mini.(link{i})(j).mass*robot.mini.(link{i})(j).length^2;
    end
end

% joint angle limits
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.mini.q1.minAngle = -pi/6;
robot.mini.q1.maxAngle = pi/2;
robot.mini.q2.minAngle = -pi/4;
robot.mini.q2.maxAngle = pi/4;
robot.mini.q3.minAngle = pi/2;
robot.mini.q3.maxAngle = -pi/2;
robot.mini.q4.minAngle = pi/2;
robot.mini.q4.maxAngle = -pi/2;


%% load in the parameters of the selected robot
quadruped = robot.(robotSelection);
quadruped.EE(1).mass = 0.2; 
quadruped.EE(2).mass = 0.2; 
% offset to hip attachment point. This translates the hip attachment point
% along the x direction of the body. Here the initial offset is set such
% that the hip is centered above the trajectory.
quadruped.hipOffset(1) = quadruped.hip(1).length;
quadruped.hipOffset(2) = -quadruped.hip(2).length;