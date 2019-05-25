function [] = plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex)
%% joint torque plots
time = 0:dt:length(Leg.(EEselection).jointTorque)*dt-dt;

% create patch to indicate stance phase


figure()
subplot(3,1,1)
hold on
    plot(time, Leg.(EEselection).jointTorque(:,1), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,1),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint torque \tau [N]');
    title({EEselection,'HAA torque'})
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off


subplot(3,1,2)
hold on
    plot(time, Leg.(EEselection).jointTorque(:,2), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,2),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint torque \tau [N]');
    title('HFE torque')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(3,1,3)
hold on
    plot(time, Leg.(EEselection).jointTorque(:,3), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,3),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    str = {'Stance', 'phase'};
    text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
    str = {'Swing', 'phase'};
    text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
    xlabel('time [s]');
    ylabel('joint torque \tau [N]')
    title('KFE torque')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

%% joint velocity plots
% for joint torques we had two time step less due to taking finite difference
time = [time time(end)+dt time(end)+2*dt];

figure()
subplot(3,1,1)
hold on
    plot(time, Leg.(EEselection).qdot(:,1), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,1),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex');
    title({EEselection,'HAA velocity'})
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(3,1,2)
hold on
    plot(time, Leg.(EEselection).qdot(:,2), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,2),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex');
    title('HFE velocity')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(3,1,3)
hold on
    plot(time, Leg.(EEselection).qdot(:,3), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,3),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    str = {'Stance', 'phase'};
    text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
    str = {'Swing', 'phase'};
    text(dt*meanTouchdownIndex.(EEselection) - 0.05, 0.75*yl(1), str,'FontSize', 16)
    xlabel('time [s]');
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex')
    title('KFE velocity')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off
