function [] = plotOptimizedJointTorque(Leg, EEselection, dt)
%% joint torque plots
time = 0:dt:length(Leg.(EEselection).jointTorque)*dt-dt;

figure()
subplot(3,1,1)
    plot(time, Leg.(EEselection).jointTorque(:,1), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,1),'b')
    ylabel('joint torque \tau [N]');
    title({EEselection,'HAA torque'})
    legend ('initial leg design', 'optimized leg design')
    grid on

subplot(3,1,2)
    plot(time, Leg.(EEselection).jointTorque(:,2), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,2),'b')
    ylabel('joint torque \tau [N]');
    title('HFE torque')
    legend ('initial leg design', 'optimized leg design')
    grid on

subplot(3,1,3)
    plot(time, Leg.(EEselection).jointTorque(:,3), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,3),'b')
    xlabel('time [s]');
    ylabel('joint torque \tau [N]')
    title('KFE torque')
    legend ('initial leg design', 'optimized leg design')
    grid on

%% joint velocity plots
% for joint torques we had one time step less due to taking finite difference
% for acceleration computation
time = [time time(end)+dt];

figure()
subplot(3,1,1)
    plot(time, Leg.(EEselection).qdot(:,1), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,1),'b')
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex');
    title({EEselection,'HAA velocity'})
    legend ('initial leg design', 'optimized leg design')
    grid on

subplot(3,1,2)
    plot(time, Leg.(EEselection).qdot(:,2), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,2),'b')
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex');
    title('HFE velocity')
    legend ('initial leg design', 'optimized leg design')
    grid on

subplot(3,1,3)
    plot(time, Leg.(EEselection).qdot(:,3), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,3),'b')
    xlabel('time [s]');
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex')
    title('KFE velocity')
    legend ('initial leg design', 'optimized leg design')
    grid on