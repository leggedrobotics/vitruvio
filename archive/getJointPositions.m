% get joint positions relative to hip attachment point based on link lengths 
% and angles obtained from inverseKinematics
function r = getJointPositions(quadruped, Leg, jointCount, EEselection, meanCyclicMotionHipEE);

% hip attachment at origin
joint = {'HAA', 'HFE', 'KFE', 'EE'};

% import link lengths
l_hip(1) = quadruped.hip(1).length;
l_thigh(1) = quadruped.thigh(1).length; 
l_shank(1) = quadruped.shank(1).length; 

l_hip(2) = quadruped.hip(2).length;
l_thigh(2) = quadruped.thigh(2).length; 
l_shank(2) = quadruped.shank(2).length; 

rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(:,2);

% pre-allocate zeros for all joint positions for each leg. HAA will stay at
% origin and other joint positions will be updated
    for j = 1:jointCount
        r_x.(joint{j}) = zeros(length(Leg.(EEselection).q),1);
        r_y.(joint{j}) = zeros(length(Leg.(EEselection).q),1);
        r_z.(joint{j}) = zeros(length(Leg.(EEselection).q),1);
    end

%% calculate position of HFE
    if (EEselection == 'LF') | (EEselection == 'RF') 
        selectFrontHind = 1; 
        hipOffsetDirection = 1;
    else selectFrontHind = 2; 
         hipOffsetDirection = -1;
    end
    
    r_x.(joint{2}) = r_x.(joint{1}) + hipOffsetDirection*l_hip(selectFrontHind)*cos(rotBodyY);
    r_y.(joint{2}) = r_y.(joint{1});
    r_z.(joint{2}) = r_z.(joint{1}) + hipOffsetDirection*l_hip(selectFrontHind)*sin(rotBodyY);

%% for each leg calculate position of KFE
    r_x.(joint{3}) = r_x.(joint{2}) + l_thigh(selectFrontHind)*cos(rotBodyY + Leg.(EEselection).q(:,2));
    r_y.(joint{3}) = r_y.(joint{2}) + l_thigh(selectFrontHind)*cos(Leg.(EEselection).q(:,2) + rotBodyY).*sin(Leg.(EEselection).q(:,1));
    r_z.(joint{3}) = r_z.(joint{2}) + l_thigh(selectFrontHind)*cos(Leg.(EEselection).q(:,2) + rotBodyY).*cos(Leg.(EEselection).q(:,1));

%% for each leg calculate position of AFE / EE
    r_x.(joint{4}) = r_x.(joint{3}) + l_shank(selectFrontHind)*(cos(rotBodyY + Leg.(EEselection).q(:,2) + Leg.(EEselection).q(:,3)));
    r_y.(joint{4}) = r_y.(joint{3}) + l_shank(selectFrontHind)*(cos(rotBodyY + Leg.(EEselection).q(:,2) + Leg.(EEselection).q(:,3))).*sin(Leg.(EEselection).q(:,1));
    r_z.(joint{4}) = r_z.(joint{3}) + l_shank(selectFrontHind)*cos(rotBodyY + Leg.(EEselection).q(:,2) + Leg.(EEselection).q(:,3)).*cos(Leg.(EEselection).q(:,1));

%% save coordinates to vector r for each leg and for each joint
    for i =1:jointCount
        r.(joint{i}) = [r_x.(joint{i}) r_y.(joint{i}) r_z.(joint{i})];
    end
