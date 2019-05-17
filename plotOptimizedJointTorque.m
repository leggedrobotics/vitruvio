%plots joint torque and velocity for initial and optimized link lengths
function [] = plotOptimizedJointTorque(jointTorque, jointTorqueOpt, EEselection, dt, q, qOpt)
time = 0:dt:length(jointTorque.(EEselection))*dt-dt;

%% joint torque plots
figure()
title('Joint torques for initial and optimized link lenghts')

subplot(3,1,1)
plot(time, jointTorque.(EEselection)(:,1), 'r', ...
     time, jointTorqueOpt.(EEselection)(:,1),'b')
ylabel('joint torque [N]');
title('HAA') 
legend ('initial leg design', 'optimized leg design')
grid on

subplot(3,1,2)
plot(time, jointTorque.(EEselection)(:,2), 'r', ...
     time, jointTorqueOpt.(EEselection)(:,2),'b')
ylabel('joint torque [N]');
title('HFE') 
legend ('initial leg design', 'optimized leg design')
grid on

subplot(3,1,3)
plot(time, jointTorque.(EEselection)(:,3), 'r', ...
     time, jointTorqueOpt.(EEselection)(:,3),'b')
xlabel('time [s]');
ylabel('joint torque [N]')
title('KFE') 
legend ('initial leg design', 'optimized leg design')
grid on

%% joint velocity plots
% for joint torques we lose one time step due to taking finite difference
% for acceleration
time = [time time(end)+dt];
figure()
title('Joint velocity for initial and optimized link lenghts')

subplot(3,1,1)
plot(time, q.(EEselection).angVel(:,1), 'r', ...
     time, qOpt.(EEselection).angVel(:,1),'b')
ylabel('joint velocity [rad/s]');
title('HAA') 
legend ('initial leg design', 'optimized leg design')
grid on

subplot(3,1,2)
plot(time, q.(EEselection).angVel(:,2), 'r', ...
     time, qOpt.(EEselection).angVel(:,2),'b')
ylabel('joint velocity [rad/s]');
title('HFE') 
legend ('initial leg design', 'optimized leg design')
grid on

subplot(3,1,3)
plot(time, q.(EEselection).angVel(:,3), 'r', ...
     time, qOpt.(EEselection).angVel(:,3),'b')
xlabel('time [s]');
ylabel('joint velocity [rad/s]')
title('KFE') 
legend ('initial leg design', 'optimized leg design')
grid on