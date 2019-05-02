%% solve q1 = qHAA

l_hip(1) = quadruped.length.hip(1);
l_thigh(1) = quadruped.length.thigh(1);
l_shank(1) = quadruped.length.shank(1);
l_foot(1) = quadruped.length.foot(1);

l_hip(2) = quadruped.length.hip(2);
l_thigh(2) = quadruped.length.thigh(2);
l_shank(2) = quadruped.length.shank(2);
l_foot(2) = quadruped.length.foot(2);

y_LF = foot_LF_bodyframe.position(:,2);

syms q1

for i = 1:length(foot_LF_bodyframe.position(:,2))
q(i,:) = solve(-l_hip(1)*sin(q1)+(l_thigh(1)+l_shank(1)+l_foot(1))*cos(q1)==y_LF(i),q1)';
i
end

q1 = real(double(q(:,2)));

%% get rotation matrix to rotate positions from qHAA frame back to hip attachment frame