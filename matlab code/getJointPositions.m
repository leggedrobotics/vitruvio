%% getJointPositions

% get joint positions based on link lengths and angles obtained from
% inverseKinematics

% r of size length(q) x number of joints x number of legs
% r = getJointPositions(quadruped, q, jointCount);

% hip attachment at origin
jointCount = 3;
EE = {'LF', 'LH', 'RF', 'RH'};
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
for i = 1:length(EE)
    for j = 1:length(joint)
        r_x.(EE{i}).(joint{j}) = zeros(length(q.(EE{i})),1);
        r_y.(EE{i}).(joint{j}) = zeros(length(q.(EE{i})),1);
        r_z.(EE{i}).(joint{j}) = zeros(length(q.(EE{i})),1);
    end
end

%% for each leg calculate position of HFE
for i = 1:length(EE)
    if (i < 3)
        j = 1; % front legs
    else j = 2; % hind legs
    end
    r_x.(EE{i}).(joint{2}) = r_x.(EE{i}).(joint{1});
    r_y.(EE{i}).(joint{2}) = r_y.(EE{i}).(joint{1}) + l_hip(j)*sin(q.(EE{i})(:,1));
    r_z.(EE{i}).(joint{2}) = r_z.(EE{i}).(joint{1}) - l_hip(j)*cos(q.(EE{i})(:,1));
end

%% for each leg calculate position of KFE
for i = 1:length(EE)
    if (i < 3)
        j = 1; % front legs
    else j = 2; % hind legs
    end
    r_x.(EE{i}).(joint{3}) = r_x.(EE{i}).(joint{2}) - l_thigh(j)*sin(q.(EE{i})(:,2));
    r_y.(EE{i}).(joint{3}) = r_y.(EE{i}).(joint{2}) + l_thigh(j)*cos(q.(EE{i})(:,2)).*sin(q.(EE{i})(:,1));
    r_z.(EE{i}).(joint{3}) = r_z.(EE{i}).(joint{2}) - l_thigh(j)*cos(q.(EE{i})(:,2)).*cos(q.(EE{i})(:,1));
end

%% for each leg calculate position of AFE / EE
for i = 1:length(EE)
    if (i < 3)
        j = 1; % front legs
    else j = 2; % hind legs
    end
    r_x.(EE{i}).(joint{4}) = r_x.(EE{i}).(joint{3}) - l_shank(j)*(sin(q.(EE{i})(:,2) + q.(EE{i})(:,3)));
    r_y.(EE{i}).(joint{4}) = r_y.(EE{i}).(joint{3}) + l_shank(j)*(cos(q.(EE{i})(:,2) + q.(EE{i})(:,3))).*sin(q.(EE{i})(:,1));
    r_z.(EE{i}).(joint{4}) = r_z.(EE{i}).(joint{3}) - l_shank(j)*cos(q.(EE{i})(:,2) + q.(EE{i})(:,3)).*cos(q.(EE{i})(:,1));
end