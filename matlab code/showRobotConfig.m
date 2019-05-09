j = 3;
EE_name = {'LF', 'LH', 'RF', 'RH'};

for i = 1:length(joint.RF.angle)
    figure(11)
config(i,:) = [0 0 0 0 0]; %joint.LF.angle(i,:)];   

jnt2.JointAxis = [1 0 0];
jnt3.JointAxis = [1 0 0];
jnt4.JointAxis = [1 0 0];
jnt5.JointAxis = [0 1 0];

    show(robot,config(i,:));
    config(i,:);
    hold on
plot3(meanCyclicMotionHipEE.(EE_name{j}).position(:,1),meanCyclicMotionHipEE.(EE_name{j}).position(:,2),meanCyclicMotionHipEE.(EE_name{j}).position(:,3),'r', 'LineWidth', 3)
hold off
end
