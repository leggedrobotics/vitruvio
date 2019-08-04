%% Nominal robot classes
% This file contains the robot properties for each of the defined robot
% classes. The function returns the properties for the selected robot.

function robotProperties = getRobotProperties(robotSelection, transmissionMethod, actuateJointDirectly, jointNames, linkNames, linkCount)

%% Your robot
%%% Add your robot properties %%%
    robot.yourRobot.mass.total = 29.5; 
    robot.yourRobot.legCount   = 4;

    % Density of each link
    robot.yourRobot.legDensity.hip(1)       = 249.4;   robot.yourRobot.legDensity.hip(2)       = 249.4;
    robot.yourRobot.legDensity.thigh(1)     = 249.4;   robot.yourRobot.legDensity.thigh(2)     = 249.4;
    robot.yourRobot.legDensity.shank(1)     = 200;     robot.yourRobot.legDensity.shank(2)     = 200;
    robot.yourRobot.legDensity.foot(1)      = 150;     robot.yourRobot.legDensity.foot(2)      = 200;
    robot.yourRobot.legDensity.phalanges(1) = 100;     robot.yourRobot.legDensity.phalanges(2) = 100;
    
    robot.yourRobot.EE(1).mass = 0.1923;
    robot.yourRobot.EE(2).mass = 0.1923;

    % offset from CoM to HAA for each leg.
    robot.yourRobot.xNom(1) = 0.34;
    robot.yourRobot.xNom(2) = 0.34;
    robot.yourRobot.yNom(1) = 0.19;
    robot.yourRobot.yNom(2) = 0.19;
    robot.yourRobot.zNom = -0.05; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.
    
    robot.yourRobot.nomHipPos.LF = [ robot.yourRobot.xNom(1),  robot.yourRobot.yNom(1), robot.yourRobot.zNom];
    robot.yourRobot.nomHipPos.LH = [-robot.yourRobot.xNom(2),  robot.yourRobot.yNom(2), robot.yourRobot.zNom];
    robot.yourRobot.nomHipPos.RF = [ robot.yourRobot.xNom(1), -robot.yourRobot.yNom(1), robot.yourRobot.zNom];
    robot.yourRobot.nomHipPos.RH = [-robot.yourRobot.xNom(2), -robot.yourRobot.yNom(2), robot.yourRobot.zNom];

    % link lengths [m]
    % fore, hind
    robot.yourRobot.hip(1).length = 0.0001;
    robot.yourRobot.hip(2).length = 0.0001;
    robot.yourRobot.thigh(1).length = 0.25;
    robot.yourRobot.thigh(2).length = 0.25;
    robot.yourRobot.shank(1).length = 0.33;
    robot.yourRobot.shank(2).length = 0.33;
    robot.yourRobot.foot(1).length = 0.15;
    robot.yourRobot.foot(2).length = 0.15;
    robot.yourRobot.phalanges(1).length = 0.1;
    robot.yourRobot.phalanges(2).length = 0.1;

    % link radius [m]
    % update these values
    robot.yourRobot.hip(1).radius = 0.05;
    robot.yourRobot.hip(2).radius = 0.05;
    robot.yourRobot.thigh(1).radius = 0.05;
    robot.yourRobot.thigh(2).radius = 0.05;
    robot.yourRobot.shank(1).radius = 0.05;
    robot.yourRobot.shank(2).radius = 0.05;
    robot.yourRobot.foot(1).radius = 0.05;
    robot.yourRobot.foot(2).radius = 0.05;
    robot.yourRobot.phalanges(1).radius = 0.05;
    robot.yourRobot.phalanges(2).radius = 0.05;

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.yourRobot.q1.minAngle = -pi;
    robot.yourRobot.q1.maxAngle = pi;
    robot.yourRobot.q2.minAngle = -2*pi;
    robot.yourRobot.q2.maxAngle = 2*pi;
    robot.yourRobot.q3.minAngle = -2*pi;
    robot.yourRobot.q3.maxAngle = 2*pi;
    robot.yourRobot.q4.minAngle = -pi;
    robot.yourRobot.q4.maxAngle = pi;
    robot.yourRobot.q5.minAngle = -pi;
    robot.yourRobot.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.yourRobot.hipOffset(1) = robot.yourRobot.hip(1).length;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    %% ANYmal
    robot.ANYmal.mass.total = 29.5; 
    robot.ANYmal.legCount   = 4;
    
    % Density of each link
    robot.ANYmal.legDensity.hip(1)       = 387.4678;  robot.ANYmal.legDensity.hip(2)       = 387.4678;
    robot.ANYmal.legDensity.thigh(1)     = 343.7730;  robot.ANYmal.legDensity.thigh(2)     = 343.7730;
    robot.ANYmal.legDensity.shank(1)     = 81.0208;   robot.ANYmal.legDensity.shank(2)     = 81.0208;
    robot.ANYmal.legDensity.foot(1)      = 80;        robot.ANYmal.legDensity.foot(2)      = 80;
    robot.ANYmal.legDensity.phalanges(1) = 80;        robot.ANYmal.legDensity.phalanges(2) = 80;    
    
    % End effector mass
    robot.ANYmal.EE(1).mass = 0.1923;
    robot.ANYmal.EE(2).mass = 0.1923;

    % offset from CoM to HAA for each leg.
    robot.ANYmal.xNom(1) = 0.34;
    robot.ANYmal.xNom(2) = 0.34;
    robot.ANYmal.yNom(1) = 0.19;
    robot.ANYmal.yNom(2) = 0.19;
    robot.ANYmal.zNom = 0; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.
    
    robot.ANYmal.nomHipPos.LF = [ robot.ANYmal.xNom(1),  robot.ANYmal.yNom(1), robot.ANYmal.zNom];
    robot.ANYmal.nomHipPos.LH = [-robot.ANYmal.xNom(2),  robot.ANYmal.yNom(2), robot.ANYmal.zNom];
    robot.ANYmal.nomHipPos.RF = [ robot.ANYmal.xNom(1), -robot.ANYmal.yNom(1), robot.ANYmal.zNom];
    robot.ANYmal.nomHipPos.RH = [-robot.ANYmal.xNom(2), -robot.ANYmal.yNom(2), robot.ANYmal.zNom];

    % link lengths [m]
    % fore, hind
    robot.ANYmal.hip(1).length = 0.14;
    robot.ANYmal.hip(2).length = 0.14;
    robot.ANYmal.thigh(1).length = 0.25;
    robot.ANYmal.thigh(2).length = 0.25;
    robot.ANYmal.shank(1).length = 0.33;
    robot.ANYmal.shank(2).length = 0.33;
    robot.ANYmal.foot(1).length = 0.15;
    robot.ANYmal.foot(2).length = 0.15;
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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.ANYmal.q1.minAngle = -pi;
    robot.ANYmal.q1.maxAngle = pi;
    robot.ANYmal.q2.minAngle = -2*pi;
    robot.ANYmal.q2.maxAngle = 2*pi;
    robot.ANYmal.q3.minAngle = -2*pi;
    robot.ANYmal.q3.maxAngle = 2*pi;
    robot.ANYmal.q4.minAngle = -pi;
    robot.ANYmal.q4.maxAngle = pi;
    robot.ANYmal.q5.minAngle = -pi;
    robot.ANYmal.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.ANYmal.hipOffset(1) = robot.ANYmal.hip(1).length;
     robot.ANYmal.hipOffset(2) = robot.ANYmal.hip(2).length;
     
     
    %% ANYmal Bear
    robot.ANYmalBear.mass.total = 38.8; 
    robot.ANYmalBear.legCount   = 4;
    
    % Density of each link
    robot.ANYmalBear.legDensity.hip(1)       = 9391;     robot.ANYmalBear.legDensity.hip(2)       = 888.2668;
    robot.ANYmalBear.legDensity.thigh(1)     = 5829;     robot.ANYmalBear.legDensity.thigh(2)     = 888.2668;
    robot.ANYmalBear.legDensity.shank(1)     = 888.2668; robot.ANYmalBear.legDensity.shank(2)     = 888.2668;
    robot.ANYmalBear.legDensity.foot(1)      = 800;      robot.ANYmalBear.legDensity.foot(2)      = 800;
    robot.ANYmalBear.legDensity.phalanges(1) = 800;      robot.ANYmalBear.legDensity.phalanges(2) = 800;    
    
    % End effector mass
    robot.ANYmalBear.EE(1).mass = 0.1402;
    robot.ANYmalBear.EE(2).mass = 0.1402;

    % offset from CoM to base hip attachment for each leg.
    robot.ANYmalBear.xNom(1) = 0.43; %0.4; % 0.44; 
    robot.ANYmalBear.xNom(2) = 0.43; 
    robot.ANYmalBear.yNom(1) = 0.112;%0.112;
    robot.ANYmalBear.yNom(2) = 0.112;%0.112;
    robot.ANYmalBear.zNom = 0; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.
    
    robot.ANYmalBear.nomHipPos.LF = [ robot.ANYmalBear.xNom(1),  robot.ANYmalBear.yNom(1), robot.ANYmalBear.zNom];
    robot.ANYmalBear.nomHipPos.LH = [-robot.ANYmalBear.xNom(2),  robot.ANYmalBear.yNom(2), robot.ANYmalBear.zNom];
    robot.ANYmalBear.nomHipPos.RF = [ robot.ANYmalBear.xNom(1), -robot.ANYmalBear.yNom(1), robot.ANYmalBear.zNom];
    robot.ANYmalBear.nomHipPos.RH = [-robot.ANYmalBear.xNom(2), -robot.ANYmalBear.yNom(2), robot.ANYmalBear.zNom];

    % link lengths [m]
    % fore, hind
    robot.ANYmalBear.hip(1).length = 0.112;
    robot.ANYmalBear.hip(2).length = 0.112;
    robot.ANYmalBear.thigh(1).length = 0.25;
    robot.ANYmalBear.thigh(2).length = 0.25;
    robot.ANYmalBear.shank(1).length = 0.33; % 0.3045;
    robot.ANYmalBear.shank(2).length = 0.33; % 0.3045;
    robot.ANYmalBear.foot(1).length = 0.15;
    robot.ANYmalBear.foot(2).length = 0.15;
    robot.ANYmalBear.phalanges(1).length = 0.1;
    robot.ANYmalBear.phalanges(2).length = 0.1;

    % link radius [m]
    % update these values
    robot.ANYmalBear.hip(1).radius = 0.015;
    robot.ANYmalBear.hip(2).radius = 0.015;
    robot.ANYmalBear.thigh(1).radius = 0.015;
    robot.ANYmalBear.thigh(2).radius = 0.015;
    robot.ANYmalBear.shank(1).radius = 0.015;
    robot.ANYmalBear.shank(2).radius = 0.015;
    robot.ANYmalBear.foot(1).radius = 0.015;
    robot.ANYmalBear.foot(2).radius = 0.015;
    robot.ANYmalBear.phalanges(1).radius = 0.015;
    robot.ANYmalBear.phalanges(2).radius = 0.015;

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.ANYmalBear.q1.minAngle = -pi;
    robot.ANYmalBear.q1.maxAngle = pi;
    robot.ANYmalBear.q2.minAngle = -2*pi;
    robot.ANYmalBear.q2.maxAngle = 2*pi;
    robot.ANYmalBear.q3.minAngle = -2*pi;
    robot.ANYmalBear.q3.maxAngle = 2*pi;
    robot.ANYmalBear.q4.minAngle = -pi;
    robot.ANYmalBear.q4.maxAngle = pi;
    robot.ANYmalBear.q5.minAngle = -pi;
    robot.ANYmalBear.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
    % Keep HAA at 0.225 from CoM to match ANYmal geometry
     robot.ANYmalBear.hipOffset(1) = robot.ANYmalBear.xNom(1)-0.225; %robot.ANYmalBear.hip(1).length + 0.1;
     robot.ANYmalBear.hipOffset(2) = robot.ANYmalBear.xNom(1)-0.225; %robot.ANYmalBear.hip(2).length + 0.1;
     
     %% Vitruvian Biped
    robot.vitruvianBiped.mass.total = 2.6423; 
    robot.vitruvianBiped.legCount   = 2;
    
    % Density of each link kg/m^3
    robot.vitruvianBiped.legDensity.hip(1)       = 1060;    
    robot.vitruvianBiped.legDensity.thigh(1)     = 1060;  
    robot.vitruvianBiped.legDensity.shank(1)     = 1060; 
    robot.vitruvianBiped.legDensity.foot(1)      = 1060;  
    robot.vitruvianBiped.legDensity.phalanges(1) = 1060; 
    
    % End effector mass
    robot.vitruvianBiped.EE(1).mass = 0.025;

    % offset from CoM to HAA for each leg.
    robot.vitruvianBiped.xNom(1) = 0.02212;
    robot.vitruvianBiped.yNom(1) = 0.1;
    robot.vitruvianBiped.zNom = 0.015; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.
    
    robot.vitruvianBiped.nomHipPos.LF = [ robot.vitruvianBiped.xNom(1),  robot.vitruvianBiped.yNom(1), robot.vitruvianBiped.zNom];
    robot.vitruvianBiped.nomHipPos.RF = [ robot.vitruvianBiped.xNom(1), -robot.vitruvianBiped.yNom(1), robot.vitruvianBiped.zNom];

    % link lengths [m]
    % fore, hind
    robot.vitruvianBiped.hip(1).length = 0.0001;
    robot.vitruvianBiped.thigh(1).length = 0.2;
    robot.vitruvianBiped.shank(1).length = 0.2;
    robot.vitruvianBiped.foot(1).length = 0.05;
    robot.vitruvianBiped.phalanges(1).length = 0.05;

    % link radius [m]
    % These were selected to get the correct mass value considering the
    % material density and link length.
    robot.vitruvianBiped.hip(1).radius = 0.0169;
    robot.vitruvianBiped.thigh(1).radius = 0.0169;
    robot.vitruvianBiped.shank(1).radius = 0.0166;
    robot.vitruvianBiped.foot(1).radius = 0.015;
    robot.vitruvianBiped.phalanges(1).radius = 0.015;

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.vitruvianBiped.q1.minAngle = -pi;
    robot.vitruvianBiped.q1.maxAngle = pi;
    robot.vitruvianBiped.q2.minAngle = -2*pi;
    robot.vitruvianBiped.q2.maxAngle = 2*pi;
    robot.vitruvianBiped.q3.minAngle = -2*pi;
    robot.vitruvianBiped.q3.maxAngle = 2*pi;
    robot.vitruvianBiped.q4.minAngle = -pi;
    robot.vitruvianBiped.q4.maxAngle = pi;
    robot.vitruvianBiped.q5.minAngle = -pi;
    robot.vitruvianBiped.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.vitruvianBiped.hipOffset(1) = robot.vitruvianBiped.hip(1).length;
     
    %% Universal
    robot.universal.mass.total = 39.53; % (with payload). This value is only used in computing CoT.
    robot.universal.legCount = 4;
    
    % Density of each link
    robot.universal.legDensity.hip(1)       = 249.4;   robot.universal.legDensity.hip(2)       = 249.4;
    robot.universal.legDensity.thigh(1)     = 249.4;   robot.universal.legDensity.thigh(2)     = 249.4;
    robot.universal.legDensity.shank(1)     = 200;     robot.universal.legDensity.shank(2)     = 200;
    robot.universal.legDensity.foot(1)      = 150;     robot.universal.legDensity.foot(2)      = 200;
    robot.universal.legDensity.phalanges(1) = 100;     robot.universal.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.universal.EE(1).mass = 0.1923;
    robot.universal.EE(2).mass = 0.1923;    

    % Offset from CoM to each hip
    robot.universal.xNom(1) = 0.34;
    robot.universal.xNom(2) = 0.34;
    robot.universal.yNom(1) = 0.19;
    robot.universal.yNom(2) = 0.19;
    robot.universal.zNom = -0.566+0.4695; % offset from CoM to hip attachment

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.universal.nomHipPos.LF = [robot.universal.xNom(1), robot.universal.yNom(1), robot.universal.zNom];
    robot.universal.nomHipPos.LH = [-robot.universal.xNom(2), robot.universal.yNom(2), robot.universal.zNom];
    robot.universal.nomHipPos.RF = [robot.universal.xNom(1), -robot.universal.yNom(1), robot.universal.zNom];
    robot.universal.nomHipPos.RH = [-robot.universal.xNom(2), -robot.universal.yNom(2), robot.universal.zNom];

    % link lengths [m]
    % fore, hind
    robot.universal.hip(1).length = 0.14;
    robot.universal.hip(2).length = 0.14;
    robot.universal.thigh(1).length = 0.5;
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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.universal.q1.minAngle = -pi;
    robot.universal.q1.maxAngle = pi;
    robot.universal.q2.minAngle = -pi/2;
    robot.universal.q2.maxAngle = pi/2;
    robot.universal.q3.minAngle = -pi;
    robot.universal.q3.maxAngle = pi;
    robot.universal.q4.minAngle = -pi;
    robot.universal.q4.maxAngle = pi;
    robot.universal.q5.minAngle = -pi;
    robot.universal.q5.maxAngle = pi;
    
    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.universal.hipOffset(1) = robot.universal.hip(1).length;
     robot.universal.hipOffset(2) = robot.universal.hip(2).length;
     
    %% Speedy 
    robot.speedy.mass.total = 22.52;
    robot.speedy.legCount   = 4;
    
    % Density of each link
    robot.speedy.legDensity.hip(1)       = 249.4;   robot.speedy.legDensity.hip(2)       = 249.4;
    robot.speedy.legDensity.thigh(1)     = 249.4;   robot.speedy.legDensity.thigh(2)     = 249.4;
    robot.speedy.legDensity.shank(1)     = 200;     robot.speedy.legDensity.shank(2)     = 200;
    robot.speedy.legDensity.foot(1)      = 150;     robot.speedy.legDensity.foot(2)      = 200;
    robot.speedy.legDensity.phalanges(1) = 100;     robot.speedy.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.speedy.EE(1).mass = 0.1923;
    robot.speedy.EE(2).mass = 0.1923;    

    % Offset from CoM to each hip
    robot.speedy.xNom(1) = 0.31;
    robot.speedy.xNom(2) = 0.31;
    robot.speedy.yNom(1) = 0.1; % front
    robot.speedy.yNom(2) = 0.14; % hind
    robot.speedy.zNom = -0.304 + 0.47;

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.speedy.nomHipPos.LF = [robot.speedy.xNom(1), robot.speedy.yNom(1), robot.speedy.zNom];
    robot.speedy.nomHipPos.LH = [-robot.speedy.xNom(2), robot.speedy.yNom(2), robot.speedy.zNom];
    robot.speedy.nomHipPos.RF = [robot.speedy.xNom(1), -robot.speedy.yNom(1), robot.speedy.zNom];
    robot.speedy.nomHipPos.RH = [-robot.speedy.xNom(2), -robot.speedy.yNom(2), robot.speedy.zNom];

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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.speedy.q1.minAngle = -pi;
    robot.speedy.q1.maxAngle = pi;
    robot.speedy.q2.minAngle = -pi/2;
    robot.speedy.q2.maxAngle = pi/2;
    robot.speedy.q3.minAngle = -pi;
    robot.speedy.q3.maxAngle = pi;
    robot.speedy.q4.minAngle = -pi;
    robot.speedy.q4.maxAngle = pi;
    robot.speedy.q5.minAngle = -pi;
    robot.speedy.q5.maxAngle = pi;
    
    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.speedy.hipOffset(1) = robot.speedy.hip(1).length;
     robot.speedy.hipOffset(2) = robot.speedy.hip(2).length;

    %% Massivo 
    robot.massivo.mass.total = 80;
    robot.massivo.legCount   = 4;
    
    % Density of each link
    robot.massivo.legDensity.hip(1)       = 249.4;   robot.massivo.legDensity.hip(2)       = 249.4;
    robot.massivo.legDensity.thigh(1)     = 249.4;   robot.massivo.legDensity.thigh(2)     = 249.4;
    robot.massivo.legDensity.shank(1)     = 200;     robot.massivo.legDensity.shank(2)     = 200;
    robot.massivo.legDensity.foot(1)      = 150;     robot.massivo.legDensity.foot(2)      = 200;
    robot.massivo.legDensity.phalanges(1) = 100;     robot.massivo.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.massivo.EE(1).mass = 0.1923;
    robot.massivo.EE(2).mass = 0.1923;

    % offset from CoM to each hip
    robot.massivo.xNom(1) = 0.276;
    robot.massivo.xNom(2) = 0.276;
    robot.massivo.yNom(1) = 0.3;
    robot.massivo.yNom(2) = 0.3;
    robot.massivo.zNom = -0.553 + 0.5043;

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.massivo.nomHipPos.LF = [robot.massivo.xNom(1), robot.massivo.yNom(1), robot.massivo.zNom];
    robot.massivo.nomHipPos.LH = [-robot.massivo.xNom(2), robot.massivo.yNom(2), robot.massivo.zNom];
    robot.massivo.nomHipPos.RF = [robot.massivo.xNom(1), -robot.massivo.yNom(1), robot.massivo.zNom];
    robot.massivo.nomHipPos.RH = [-robot.massivo.xNom(2), -robot.massivo.yNom(2), robot.massivo.zNom];

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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.massivo.q1.minAngle = -pi;
    robot.massivo.q1.maxAngle = pi;
    robot.massivo.q2.minAngle = -pi/2;
    robot.massivo.q2.maxAngle = pi/2;
    robot.massivo.q3.minAngle = -pi;
    robot.massivo.q3.maxAngle = pi;
    robot.massivo.q4.minAngle = -pi;
    robot.massivo.q4.maxAngle = pi;
    robot.massivo.q5.minAngle = -pi;
    robot.massivo.q5.maxAngle = pi;
    
    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.massivo.hipOffset(1) = robot.massivo.hip(1).length;
     robot.massivo.hipOffset(2) = -robot.massivo.hip(2).length;
     
    %% Centaur 
    robot.centaur.mass.total = 80;
    robot.centaur.legCount   = 4;

    % Density of each link
    robot.centaur.legDensity.hip(1)       = 249.4;   robot.centaur.legDensity.hip(2)       = 249.4;
    robot.centaur.legDensity.thigh(1)     = 249.4;   robot.centaur.legDensity.thigh(2)     = 249.4;
    robot.centaur.legDensity.shank(1)     = 200;     robot.centaur.legDensity.shank(2)     = 200;
    robot.centaur.legDensity.foot(1)      = 150;     robot.centaur.legDensity.foot(2)      = 200;
    robot.centaur.legDensity.phalanges(1) = 100;     robot.centaur.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.centaur.EE(1).mass = 0.1923;
    robot.centaur.EE(2).mass = 0.1923;

    % offset from CoM to each hip
    robot.centaur.xNom(1) = 0.1451;
    robot.centaur.xNom(2) = 0.407;
    robot.centaur.yNom(1) = 0.3;
    robot.centaur.yNom(2) = 0.3;
    robot.centaur.zNom = -0.553 + 0.5043;

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.centaur.nomHipPos.LF = [robot.centaur.xNom(1), robot.centaur.yNom(1), robot.centaur.zNom];
    robot.centaur.nomHipPos.LH = [-robot.centaur.xNom(2), robot.centaur.yNom(2), robot.centaur.zNom];
    robot.centaur.nomHipPos.RF = [robot.centaur.xNom(1), -robot.centaur.yNom(1), robot.centaur.zNom];
    robot.centaur.nomHipPos.RH = [-robot.centaur.xNom(2), -robot.centaur.yNom(2), robot.centaur.zNom];

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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.centaur.q1.minAngle = -pi;
    robot.centaur.q1.maxAngle = pi;
    robot.centaur.q2.minAngle = -pi/2;
    robot.centaur.q2.maxAngle = pi/2;
    robot.centaur.q3.minAngle = -pi;
    robot.centaur.q3.maxAngle = pi;
    robot.centaur.q4.minAngle = -pi;
    robot.centaur.q4.maxAngle = pi;
    robot.centaur.q5.minAngle = -pi;
    robot.centaur.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.centaur.hipOffset(1) = robot.centaur.hip(1).length;
     robot.centaur.hipOffset(2) = robot.centaur.hip(2).length;    
    
    %% Mini 
    robot.mini.mass.total = 10;
    robot.mini.legCount   = 4;

    % Density of each link
    robot.mini.legDensity.hip(1)       = 249.4;   robot.mini.legDensity.hip(2)       = 249.4;
    robot.mini.legDensity.thigh(1)     = 249.4;   robot.mini.legDensity.thigh(2)     = 249.4;
    robot.mini.legDensity.shank(1)     = 200;     robot.mini.legDensity.shank(2)     = 200;
    robot.mini.legDensity.foot(1)      = 150;     robot.mini.legDensity.foot(2)      = 200;
    robot.mini.legDensity.phalanges(1) = 100;     robot.mini.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.mini.EE(1).mass = 0.1923;
    robot.mini.EE(2).mass = 0.1923;
    
    robot.mini.xNom(1) = 0.18;
    robot.mini.xNom(2) = 0.18;
    robot.mini.yNom(1) = 0.1;
    robot.mini.yNom(2) = 0.1;
    robot.mini.zNom = -0.186 + 0.198;

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.mini.nomHipPos.LF = [robot.mini.xNom(1), robot.mini.yNom(1), robot.mini.zNom];
    robot.mini.nomHipPos.LH = [-robot.mini.xNom(2), robot.mini.yNom(2), robot.mini.zNom];
    robot.mini.nomHipPos.RF = [robot.mini.xNom(1), -robot.mini.yNom(1), robot.mini.zNom];
    robot.mini.nomHipPos.RH = [-robot.mini.xNom(2), -robot.mini.yNom(2), robot.mini.zNom];

    % link lengths [m]
    % fore, hind
    robot.mini.hip(1).length = 0.15;
    robot.mini.hip(2).length = 0.15;
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

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.mini.q1.minAngle = -pi;
    robot.mini.q1.maxAngle = pi;
    robot.mini.q2.minAngle = -pi/2;
    robot.mini.q2.maxAngle = pi/2;
    robot.mini.q3.minAngle = -pi;
    robot.mini.q3.maxAngle = pi;
    robot.mini.q4.minAngle = -pi;
    robot.mini.q4.maxAngle = pi;
    robot.mini.q5.minAngle = -pi;
    robot.mini.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.mini.hipOffset(1) = robot.mini.hip(1).length;
     robot.mini.hipOffset(2) = robot.mini.hip(2).length;    
    
    %% defaultHopper 
    robot.defaultHopper.mass.total = 10;
    robot.defaultHopper.legCount   = 1;
    
    % Density of each link
    robot.defaultHopper.legDensity.hip(1)       = 249.4;   robot.defaultHopper.legDensity.hip(2)       = 249.4;
    robot.defaultHopper.legDensity.thigh(1)     = 249.4;   robot.defaultHopper.legDensity.thigh(2)     = 249.4;
    robot.defaultHopper.legDensity.shank(1)     = 200;     robot.defaultHopper.legDensity.shank(2)     = 200;
    robot.defaultHopper.legDensity.foot(1)      = 150;     robot.defaultHopper.legDensity.foot(2)      = 200;
    robot.defaultHopper.legDensity.phalanges(1) = 100;     robot.defaultHopper.legDensity.phalanges(2) = 100;
    
    % End effector mass
    robot.defaultHopper.EE(1).mass = 0.1923;

    robot.defaultHopper.xNom(1) = 0;
    robot.defaultHopper.yNom(1) = 0;
    robot.defaultHopper.zNom =  0;

    % row order:    LF LH RF RH
    % column order: x, y, z
    robot.defaultHopper.nomHipPos.LF = [robot.defaultHopper.xNom(1), robot.defaultHopper.yNom(1), robot.defaultHopper.zNom];

    % link lengths [m]
    robot.defaultHopper.hip(1).length = 0.01;
    robot.defaultHopper.thigh(1).length = 0.4;
    robot.defaultHopper.shank(1).length = 0.4;
    robot.defaultHopper.foot(1).length = 0.05;
    robot.defaultHopper.phalanges(1).length = 0.05;

    % link radius [m]
    robot.defaultHopper.hip(1).radius = 0.02;
    robot.defaultHopper.thigh(1).radius = 0.02;
    robot.defaultHopper.shank(1).radius = 0.03;
    robot.defaultHopper.foot(1).radius = 0.03;
    robot.defaultHopper.phalanges(1).radius = 0.03;

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.defaultHopper.q1.minAngle = -pi;
    robot.defaultHopper.q1.maxAngle = pi;
    robot.defaultHopper.q2.minAngle = -pi/2;
    robot.defaultHopper.q2.maxAngle = pi/2;
    robot.defaultHopper.q3.minAngle = -pi;
    robot.defaultHopper.q3.maxAngle = pi;
    robot.defaultHopper.q4.minAngle = -pi;
    robot.defaultHopper.q4.maxAngle = pi;
    robot.defaultHopper.q5.minAngle = -pi;
    robot.defaultHopper.q5.maxAngle = pi;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HAA is at the location specified above by nomHipPos.
     robot.defaultHopper.hipOffset(1) = robot.defaultHopper.hip(1).length;
     
     
    %% Compute link mass and inertia for selected robot
    
    % Additional mass due to transmission
    [transmissionMass, transmissionGearRatio] = getTransmissionProperties(transmissionMethod, actuateJointDirectly, robot, robotSelection, jointNames, linkNames, linkCount);
    
        % Link mass [kg] and inertia [kg.m^2] based on cylindrical link
        % with constant density. Density is dependent on link density and
        % transmission mass.
        
    for i = 1:linkCount+1
        if robot.(robotSelection).legCount > 2
            frontHindIndex = 2; % Robot has front and hind legs.
        else
            frontHindIndex = 1; % Robot only has 'front' legs.
        end
        for j = 1:frontHindIndex % (1 = front, 2 = hind)
            % M = pi*R^2*L_link*density_link + m_transmission
            robot.(robotSelection).(linkNames{i})(j).mass = pi()*robot.(robotSelection).(linkNames{i})(j).radius^2*robot.(robotSelection).(linkNames{i})(j).length*robot.(robotSelection).legDensity.(linkNames{i})(j) ...
                                                            + transmissionMass.(linkNames{i})(j);
            % Inertia = (1/3)*M*L_link^2                                            
            robot.(robotSelection).(linkNames{i})(j).inertia = (1/3)*robot.(robotSelection).(linkNames{i})(j).mass*robot.(robotSelection).(linkNames{i})(j).length^2;
        end
    end
    
    for i = 1:length(jointNames)
        if robot.(robotSelection).legCount > 2
            frontHindIndex = 2; % Robot has front and hind legs.
        else
            frontHindIndex = 1; % Robot only has 'front' legs.
        end
        for j = 1:frontHindIndex % (1 = front, 2 = hind)
            robot.(robotSelection).transmissionGearRatio.(jointNames(i,:))(j) = transmissionGearRatio.(jointNames(i,:))(j);
        end
    end
    
    %% Load in the parameters of the selected robot into the struct robotProperties which is then used by the rest of the program  
    robotProperties = robot.(robotSelection);
end