%% Nominal robot classes
% the link lengths need to be updated during the optimization so this data
% cannot be saved into a .mat file like the motion data

function quadruped = getQuadrupedProperties(robotSelection, linkCount)

    %% ANYmal
    robot.ANYmal.mass.total = 29.5; 
    robot.ANYmal.legDensity = 249.4; %kg/m^3. Assuming legs are constant density cylinders.
    robot.ANYmal.EE(1).mass = 0.19;
    robot.ANYmal.EE(2).mass = 0.19;

    % offset from CoM to HAA for each leg.
    robot.ANYmal.xNom(1) = 0.34;
    robot.ANYmal.xNom(2) = 0.34;
    robot.ANYmal.yNom(1) = 0.19;
    robot.ANYmal.yNom(2) = 0.19;
    robot.ANYmal.zNom = 0.05; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.
    
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

    % Link mass [kg] and inertia [kg.m^2] based on cylindrical link with constant density
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
    robot.ANYmal.q5.minAngle = -pi/2;
    robot.ANYmal.q5.maxAngle = pi/2;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.ANYmal.hipOffset(1) = robot.ANYmal.hip(1).length;
     robot.ANYmal.hipOffset(2) = robot.ANYmal.hip(2).length;
     
    %% Universal
    robot.universal.mass.total = 39.53; % with payload
    robot.universal.legDensity = 249.4;
    robot.universal.EE(1).mass = 0.19;
    robot.universal.EE(2).mass = 0.19;    

    % offset from CoM to each hip
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

    %link mass [kg] based on cylindrical link with constant density
    for i = 1:length(link)
        for j = 1:2
            robot.universal.(link{i})(j).mass = pi()*robot.universal.(link{i})(j).radius^2*robot.universal.(link{i})(j).length*robot.universal.legDensity;
            robot.universal.(link{i})(j).inertia = (1/3)*robot.universal.(link{i})(j).mass*robot.universal.(link{i})(j).length^2;
        end
    end

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.universal.q1.minAngle = -2*pi;
    robot.universal.q1.maxAngle = 2*pi;
    robot.universal.q2.minAngle = -pi/3;
    robot.universal.q2.maxAngle = pi/3;
    robot.universal.q3.minAngle = -2*pi;
    robot.universal.q3.maxAngle = 2*pi;
    robot.universal.q4.minAngle = -2*pi;
    robot.universal.q4.maxAngle = 2*pi;
    robot.universal.q5.minAngle = -2*pi;
    robot.universal.q5.maxAngle = 2*pi;
    
    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.universal.hipOffset(1) = robot.universal.hip(1).length;
     robot.universal.hipOffset(2) = robot.universal.hip(2).length;
     
    %% Speedy 
    robot.speedy.mass.total = 22.52;
    robot.speedy.legDensity = 249.4;
    robot.speedy.EE(1).mass = 0.19;
    robot.speedy.EE(2).mass = 0.19;    

    % offset from CoM to each hip
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

    %link mass [kg] based on cylindrical link with constant density
    for i = 1:length(link)
        for j = 1:2
            robot.speedy.(link{i})(j).mass = pi()*robot.speedy.(link{i})(j).radius^2*robot.speedy.(link{i})(j).length*robot.speedy.legDensity;
            robot.speedy.(link{i})(j).inertia = (1/3)*robot.speedy.(link{i})(j).mass*robot.speedy.(link{i})(j).length^2;
        end
    end

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.speedy.q1.minAngle = -pi;
    robot.speedy.q1.maxAngle = pi;
    robot.speedy.q2.minAngle = -pi;
    robot.speedy.q2.maxAngle = pi;
    robot.speedy.q3.minAngle = -pi;
    robot.speedy.q3.maxAngle = pi;
    robot.speedy.q4.minAngle = -pi;
    robot.speedy.q4.maxAngle = pi;
    
    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.speedy.hipOffset(1) = robot.speedy.hip(1).length;
     robot.speedy.hipOffset(2) = robot.speedy.hip(2).length;

    %% Massivo 
    robot.massivo.mass.total = 80;
    robot.massivo.legDensity = 249.4;
    robot.massivo.EE(1).mass = 0.19;
    robot.massivo.EE(2).mass = 0.19;

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

    %link mass [kg] based on cylindrical link with constant density
    for i = 1:length(link)
        for j = 1:2
            robot.massivo.(link{i})(j).mass = pi()*robot.massivo.(link{i})(j).radius^2*robot.massivo.(link{i})(j).length*robot.massivo.legDensity;
            robot.massivo.(link{i})(j).inertia = (1/3)*robot.massivo.(link{i})(j).mass*robot.massivo.(link{i})(j).length^2;
        end
    end

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
    robot.centaur.legDensity = 249.4;
    robot.centaur.EE(1).mass = 0.19;
    robot.centaur.EE(2).mass = 0.19;

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
    robot.centaur.q3.minAngle = -pi/2;
    robot.centaur.q3.maxAngle = pi/2;
    robot.centaur.q4.minAngle = -pi/2;
    robot.centaur.q4.maxAngle = pi/2;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.centaur.hipOffset(1) = robot.centaur.hip(1).length;
     robot.centaur.hipOffset(2) = robot.centaur.hip(2).length;    
    
    %% Mini 
    robot.mini.mass.total = 10;
    robot.mini.legDensity = 249.4;
    robot.mini.EE(1).mass = 0.19;
    robot.mini.EE(2).mass = 0.19;
    
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
    robot.mini.q3.minAngle = -pi/2;
    robot.mini.q3.maxAngle = pi/2;
    robot.mini.q4.minAngle = -pi/2;
    robot.mini.q4.maxAngle = pi/2;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HFE is at the location specified above by nomHipPos.
     robot.mini.hipOffset(1) = robot.mini.hip(1).length;
     robot.mini.hipOffset(2) = robot.mini.hip(2).length;    
    
    %% defaultHopper 
    robot.defaultHopper.mass.total = 10;
    robot.defaultHopper.legDensity = 249.4;
    robot.defaultHopper.EE(1).mass = 0.19;

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

    %link mass [kg] based on cylindrical link with constant density
    for i = 1:length(link)
        for j = 1:1 % only 1 leg for hopper
            robot.defaultHopper.(link{i})(j).mass = pi()*robot.defaultHopper.(link{i})(j).radius^2*robot.defaultHopper.(link{i})(j).length*robot.defaultHopper.legDensity;
            robot.defaultHopper.(link{i})(j).inertia = (1/3)*robot.defaultHopper.(link{i})(j).mass*robot.defaultHopper.(link{i})(j).length^2;
        end
    end

    % joint angle limits
    % q1 HAA, q2 HFE, q3 KFE, q4 AFE
    robot.defaultHopper.q1.minAngle = -2*pi;
    robot.defaultHopper.q1.maxAngle = 2*pi;
    robot.defaultHopper.q2.minAngle = -2*pi;
    robot.defaultHopper.q2.maxAngle = 2*pi;
    robot.defaultHopper.q3.minAngle = -pi/2;
    robot.defaultHopper.q3.maxAngle = pi/2;
    robot.defaultHopper.q4.minAngle = -pi/2;
    robot.defaultHopper.q4.maxAngle = pi/2;

    % Hip offset. This shifts HAA along the body x direction. It is
    % an optimization parameter and is initially set to the hip length such
    % that HAA is at the location specified above by nomHipPos.
     robot.defaultHopper.hipOffset(1) = robot.defaultHopper.hip(1).length;
     
    %% load in the parameters of the selected robot into the struct quadruped which is then used by the rest of the program
    quadruped = robot.(robotSelection);
end