function [] = plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
%% joint torque plots
time = 0:dt:length(Leg.(EEselection).jointTorque)*dt-dt;

subplotCount = linkCount+1;
figure()
subplot(subplotCount,1,1)
hold on
    plot(time, Leg.(EEselection).jointTorque(:,1), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,1),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint torque \tau [N]');
    title({taskSelection, EEselection,'HAA torque'})
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(subplotCount,1,2)
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

subplot(subplotCount,1,3)
    hold on
    plot(time, Leg.(EEselection).jointTorque(:,3), 'r', ...
         time, Leg.(EEselection).jointTorqueOpt(:,3),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    if (linkCount == 2)
        str = {'Stance', 'phase'};
        text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
        str = {'Swing', 'phase'};
        text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
    end
    xlabel('time [s]');
    ylabel('joint torque \tau [N]')
    title('KFE torque')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

if (linkCount == 3) || (linkCount == 4)
        subplot(subplotCount,1,4)
            hold on
            plot(time, Leg.(EEselection).jointTorque(:,4), 'r', ...
                 time, Leg.(EEselection).jointTorqueOpt(:,4),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 3)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
            end
            ylabel('joint torque \tau [N]');
            title('AFE torque')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end

if (linkCount == 4)
        subplot(subplotCount,1,5)
            hold on
            plot(time, Leg.(EEselection).jointTorque(:,5), 'r', ...
                 time, Leg.(EEselection).jointTorqueOpt(:,5),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 4)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
            end
            ylabel('joint torque \tau [N]');
            title('DFE torque')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end
%% joint velocity plots
% for joint torques we had two time step less due to taking finite difference
time = [time time(end)+dt time(end)+2*dt];

figure()
subplot(subplotCount,1,1)
hold on
    plot(time, Leg.(EEselection).qdot(:,1), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,1),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex');
    title({taskSelection, EEselection,'HAA velocity'})
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(subplotCount,1,2)
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

subplot(subplotCount,1,3)
hold on
    plot(time, Leg.(EEselection).qdot(:,3), 'r', ...
         time, Leg.(EEselection).qdotOpt(:,3),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    if (linkCount == 2)
        str = {'Stance', 'phase'};
        text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
        str = {'Swing', 'phase'};
        text(dt*meanTouchdownIndex.(EEselection) - 0.05, 0.75*yl(1), str,'FontSize', 16)
    end
    xlabel('time [s]');
    ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex')
    title('KFE velocity')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

if (linkCount == 3) || (linkCount == 4)
        subplot(subplotCount,1,4)
            hold on
            plot(time, Leg.(EEselection).qdot(:,4), 'r', ...
                 time, Leg.(EEselection).qdotOpt(:,4),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 3)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05, 0.75*yl(1), str,'FontSize', 16)
            end            
            xlabel('time [s]');
            ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex')
            title('AFE velocity')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end

if (linkCount == 4)
        subplot(subplotCount,1,5)
            hold on
            plot(time, Leg.(EEselection).qdot(:,5), 'r', ...
                 time, Leg.(EEselection).qdotOpt(:,5),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 4)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05, 0.75*yl(1), str,'FontSize', 16)
            end            
            xlabel('time [s]');
            ylabel('joint velocity $\dot{q}$ [rad/s]','interpreter','latex')
            title('DFE velocity')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end

%% joint power plots
time = 0:dt:length(Leg.(EEselection).jointPower)*dt-dt;

subplotCount = linkCount+1;
figure()
subplot(subplotCount,1,1)
hold on
    plot(time, Leg.(EEselection).jointPower(:,1), 'r', ...
         time, Leg.(EEselection).jointPowerOpt(:,1),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint power \tau [W]');
    title({taskSelection, EEselection,'HAA power'})
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(subplotCount,1,2)
hold on
    plot(time, Leg.(EEselection).jointPower(:,2), 'r', ...
         time, Leg.(EEselection).jointPowerOpt(:,2),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    ylabel('joint power \tau [W]');
    title('HFE power')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

subplot(subplotCount,1,3)
    hold on
    plot(time, Leg.(EEselection).jointPower(:,3), 'r', ...
         time, Leg.(EEselection).jointPowerOpt(:,3),'b')
    yl = ylim;
    patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
    alpha(0.05);
    if (linkCount == 2)
        str = {'Stance', 'phase'};
        text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
        str = {'Swing', 'phase'};
        text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
    end
    xlabel('time [s]');
    ylabel('joint power \tau [W]')
    title('KFE power')
    legend ('initial leg design', 'optimized leg design')
    grid on
hold off

if (linkCount == 3) || (linkCount == 4)
        subplot(subplotCount,1,4)
            hold on
            plot(time, Leg.(EEselection).jointPower(:,4), 'r', ...
                 time, Leg.(EEselection).jointPowerOpt(:,4),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 3)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
            end
            ylabel('joint power \tau [W]');
            title('AFE power')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end

if (linkCount == 4)
        subplot(subplotCount,1,5)
            hold on
            plot(time, Leg.(EEselection).jointPower(:,5), 'r', ...
                 time, Leg.(EEselection).jointPowerOpt(:,5),'b')
            yl = ylim;
            patch([dt*meanTouchdownIndex.(EEselection) dt*meanTouchdownIndex.(EEselection) time(end) time(end)],[yl(1) yl(2) yl(2) yl(1)], 'b')
            alpha(0.05);
            if (linkCount == 4)
                str = {'Stance', 'phase'};
                text(time(end) - 0.05,0.75*yl(1),str,'FontSize',16)
                str = {'Swing', 'phase'};
                text(dt*meanTouchdownIndex.(EEselection) - 0.05,0.75*yl(1),str,'FontSize',16)
            end
            ylabel('joint power \tau [W]');
            title('DFE power')
            legend ('initial leg design', 'optimized leg design')
            grid on
        hold off
end