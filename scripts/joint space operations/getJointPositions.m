%% getJointPositions

% get joint positions based on link lengths and angles obtained from
% inverseKinematics

% r of size length(q) x number of joints x number of legs
function r = getJointPositions(quadruped, q, jointCount, EEselection);

% hip attachment at origin
joint = {'HAA', 'HFE', 'KFE', 'EE'};

% import link lengths
l_hip(1) = quadruped.hip(1).length;
l_thigh(1) = quadruped.thigh(1).length; 
l_shank(1) = quadruped.shank(1).length; 

l_hip(2) = quadruped.hip(2).length;
l_thigh(2) = quadruped.thigh(2).length; 
l_shank(2) = quadruped.shank(2).length; 

% pre-allocate zeros for all joint positions for each leg. HAA will stay at
% origin and other joint positions will be updated
    for j = 1:jointCount
        r_x.(EEselection).(joint{j}) = zeros(length(q.(EEselection).angle),1);
        r_y.(EEselection).(joint{j}) = zeros(length(q.(EEselection).angle),1);
        r_z.(EEselection).(joint{j}) = zeros(length(q.(EEselection).angle),1);
    end

%% calculate position of HFE
    if (EEselection == 'LF') | (EEselection == 'RF') 
        j = 1; % front legs
    else j = 2; % hind legs
    end

    r_x.(EEselection).(joint{2}) = r_x.(EEselection).(joint{1});
    r_y.(EEselection).(joint{2}) = r_y.(EEselection).(joint{1}) + l_hip(j)*sin(q.(EEselection).angle(:,1));
    r_z.(EEselection).(joint{2}) = r_z.(EEselection).(joint{1}) - l_hip(j)*cos(q.(EEselection).angle(:,1));

%% for each leg calculate position of KFE


    r_x.(EEselection).(joint{3}) = r_x.(EEselection).(joint{2}) - l_thigh(j)*sin(q.(EEselection).angle(:,2));
    r_y.(EEselection).(joint{3}) = r_y.(EEselection).(joint{2}) + l_thigh(j)*cos(q.(EEselection).angle(:,2)).*sin(q.(EEselection).angle(:,1));
    r_z.(EEselection).(joint{3}) = r_z.(EEselection).(joint{2}) - l_thigh(j)*cos(q.(EEselection).angle(:,2)).*cos(q.(EEselection).angle(:,1));


%% for each leg calculate position of AFE / EE

    r_x.(EEselection).(joint{4}) = r_x.(EEselection).(joint{3}) - l_shank(j)*(sin(q.(EEselection).angle(:,2) + q.(EEselection).angle(:,3)));
    r_y.(EEselection).(joint{4}) = r_y.(EEselection).(joint{3}) + l_shank(j)*(cos(q.(EEselection).angle(:,2) + q.(EEselection).angle(:,3))).*sin(q.(EEselection).angle(:,1));
    r_z.(EEselection).(joint{4}) = r_z.(EEselection).(joint{3}) - l_shank(j)*cos(q.(EEselection).angle(:,2) + q.(EEselection).angle(:,3)).*cos(q.(EEselection).angle(:,1));


%% save coordinates to vector r for each leg and for each joint
    for j =1:jointCount
        r.(EEselection).(joint{j}) = [r_x.(EEselection).(joint{j}) r_y.(EEselection).(joint{j}) r_z.(EEselection).(joint{j})];
    end
