%% compare actuator torques for initial and optimized design

time = 0:dt:length(initialJointTorque.(EEselection))*dt-dt;
figure()
title('Joint torques for initial and optimized link lenghts')

subplot(3,1,1)
plot(time, initialJointTorque.(EEselection)(:,1), 'r', ...
     time, optimizedJointTorque.(EEselection)(:,1),'b')
ylabel('joint torque [N]');
title('HAA') 
legend ('initial leg design', 'optimized leg design')


subplot(3,1,2)
plot(time, initialJointTorque.(EEselection)(:,2), 'r', ...
     time, optimizedJointTorque.(EEselection)(:,2),'b')
ylabel('joint torque [N]');
title('HFE') 
legend ('initial leg design', 'optimized leg design')
 
subplot(3,1,3)
plot(time, initialJointTorque.(EEselection)(:,3), 'r', ...
     time, optimizedJointTorque.(EEselection)(:,3),'b')
xlabel('time [s]');
ylabel('joint torque [N]')
title('KFE') 
legend ('initial leg design', 'optimized leg design')

