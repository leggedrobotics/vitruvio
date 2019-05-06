clear  % Clears command history
clc   % Clears command window
clf  % Clears figure window
%========= Sets initial parameters for pendulum ===========================
g = 9.81;  % Gravity (ms-2)
l = 4;  % pendulum length (m)
initialangle1 = pi/2;  % Initial angle 1
initialangle2 = 0;   % Initial angle 2
%====== Sets x and y coordinates of pendulum top  =========================
pendulumtopx = 0;
pendulumtopy = l;
fprintf('Single pendulum simulation by James Adams \n\n')
choice = input('Press 1 for a phase portrait or 2 for a time serie plot : ');
iterations = 1; % Sets initial iteration count to 1
pausetime = 0.1;  % Pauses animation for this time
runtime = 50;  % Runs simulations for this time
tx = 0;  % Ensures time series plot remains in the figure window
%============== Solves simple pendulum differential equations =============
deq1=@(t,x) [x(2); -g/l * sin(x(1))]; % Pendulum equations uncoupled
[t,sol] = ode45(deq1,[0 runtime],[initialangle1 initialangle2]);  % uses a numerical ode solver
sol1 = sol(:,1)'; % takes the transpose for plots
sol2 = sol(:,2)';
arraysize = size(t);  % Defines array size of time intervals
timestep = t(runtime) - t(runtime-1);  % Calculates the time step of these intervals
cartesianx = l*sin(sol1);  % Converts angles into cartesian coordinates
cartesiany = l*cos(sol2);  
    
%============== plots results at each time interval =======================
for i = 1 : max(arraysize)
    subplot(2,1,1)
    plotarrayx = [pendulumtopx cartesianx(iterations)];
    plotarrayy = [pendulumtopy cartesiany(iterations)];
    plot(cartesianx(iterations),cartesiany(iterations),'ko',plotarrayx,plotarrayy,'r-')
    title(['Simple pendulum simulation            \theta = ' num2str(sol1(iterations))],'fontsize',12)
    xlabel('x [m]','fontsize',12)
    ylabel('y [m]','fontsize',12)
    axis([min(cartesianx) max(cartesianx) min(cartesiany) max(cartesiany)])
    subplot(2,1,2)
    
    % Plots either a phase portrait or time series depending on choice
    if choice == 1
        plot(sol1(iterations),sol2(iterations),'bo')
        hold on
        title('Simple pendulum phase portrait','fontsize',12)
        xlabel('\theta1','fontsize',12)
        ylabel('\theta2','fontsize',12)
        axis([min(sol1) max(sol1) min(sol2) max(sol2)])
    elseif choice == 2
        plot(t(iterations),sol1(iterations),'bo')
        title(['Simple pendulum time series for \theta1       t = ' num2str(t(iterations))],'fontsize',12)
        xlabel('t [seconds]','fontsize',12)
        ylabel('\theta1','fontsize',12)
        hold on  % Holds previous values
        axis([0 t(iterations)+(t(2)-t(1)) min(sol1) max(sol1)])
        tx = tx + timestep;  % Aligns results with the figure window
    end
    pause(pausetime)  % Shows results at each time interval
    iterations = iterations + 1;  % increases iteration count by 1
end
%=========================== End of program ===============================
