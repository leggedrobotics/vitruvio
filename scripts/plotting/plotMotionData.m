%% Plot trajectory data
function [] = plotMotionData(data, saveFiguresToPDF)

    t                    = data.time;
    tUninterpolated      = data.timeUninterpolated;
    base                 = data.base; % base motion for each leg during its cycle
    base.fullTrajectory  = data.fullTrajectory.base;
    force.fullTrajectory = data.fullTrajectory.force;
    legCount             = data.basicProperties.legCount;
    linkCount            = data.basicProperties.linkCount;
    EEnames              = data.basicProperties.EEnames;
    removalRatioStart    = data.basicProperties.trajectory.removalRatioStart;
    removalRatioEnd      = data.basicProperties.trajectory.removalRatioEnd;
    inertialEEposition   = data.inertialFrame.EEposition;
    
    if saveFiguresToPDF
        outerPosition = [0 0 1 1]; % Fullscreen
    else
        outerPosition = [0.5 0.5 0.5 0.5]; % Top right corner
    end
    
    startIndexFullTrajectory = round(length(data.fullTrajectory.r.EEdes.LF)*removalRatioStart);
    startIndexFullTrajectory(startIndexFullTrajectory<1) = 1;
    endIndexFullTrajectory = round(length(data.fullTrajectory.r.EEdes.LF)*(1-removalRatioEnd));

    %% Base position and velocity in inertial frame with sampled rectange.
    figure('name', 'Base position and velocity', 'units','normalized','outerposition',outerPosition)
    set(gcf,'color','w')
    subplot(3,2,1);
    plot(t, base.fullTrajectory.position(:,1));
    ylabel('x position [m]');
    grid on
    title('Center of mass position')
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
        
        
    subplot(3,2,3);
    hold on
    plot(t, base.fullTrajectory.position(:,2));
    ylabel('y position [m]');
    grid on
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
    hold off
    
    subplot(3,2,5);
    plot(t, base.fullTrajectory.position(:,3));
    xlabel('time [s]');
    ylabel('z position [m]');
    grid on
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
    
    subplot(3,2,2);
    plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,1));
    ylabel('x velocity [m/s]');
    grid on
    title('Center of mass velocity')
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            

    subplot(3,2,4);
    plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,2));
    ylabel('y velocity [m/s]');
    grid on
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            

    subplot(3,2,6);
    plot(t(1:length(base.fullTrajectory.velocity)), base.fullTrajectory.velocity(:,3));
    xlabel('time [s]');
    ylabel('z velocity [m/s]');
    grid on
    %Plot rectangle for sampled region
    yBounds = ylim;
    rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
    rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
    rectanglePos.height     = yBounds(2) - yBounds(1);
    rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end

    
    %% End effector position in inertial frame x vs t and z vs t
    figure('name', 'End effector position inertial frame', 'units','normalized','outerposition',outerPosition) 
    set(gcf,'color','w')
    lineColor = {'b', 'g', 'r', 'c'};
    minValue1 = []; maxValue1 = [];
    minValue2 = []; maxValue2 = [];
    minValue3 = []; maxValue3 = [];
    
    for i = 1:legCount
        EEselection = EEnames(i,:);
        subplot(3,1,1)
        hold on
            p(i) = plot(tUninterpolated, inertialEEposition.(EEselection)(:,1), lineColor{i}, 'DisplayName', EEselection);
            title('End effector position x')
            xlabel('time [s]')    
            ylabel('EE x position relative to CoM starting point [m]')
            grid on
        subplot(3,1,2) 
        hold on
            plot(tUninterpolated, inertialEEposition.(EEselection)(:,2), lineColor{i}, 'DisplayName', EEselection);
            title('End effector position y')
            xlabel('time [s]')    
            ylabel('EE y position relative to CoM starting point [m]')
            grid on
        subplot(3,1,3)
        hold on
            plot(tUninterpolated, inertialEEposition.(EEselection)(:,3), lineColor{i}, 'DisplayName', EEselection);
            title('End effector position z')
            xlabel('time [s]')    
            ylabel('EE z position relative to CoM starting point [m]')
            grid on
    end
    hold off
           
    if legCount > 1 % only display legend if there are multiple legs
        lgd = legend(p);
        lgd.FontSize = 14;
    end
    
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    %% End effector forces.
    figure('name', 'End effector forces', 'units','normalized','outerposition',outerPosition) 
    set(gcf,'color','w')
    lineColor = {'b', 'g', 'r', 'c'};
    minValue1 = []; maxValue1 = [];
    minValue2 = []; maxValue2 = [];
    minValue3 = []; maxValue3 = [];
    for i = 1:legCount
        subplot(3,1,1)
        hold on
        EEselection = EEnames(i,:);
        p(i) = plot(t, force.fullTrajectory.(EEselection)(1:length(t),1), lineColor{i}, 'DisplayName', EEselection);
        title('End effector forces')
        xlabel('time [s]')    
        ylabel('force in x-direction [N]')
        grid on
        %Plot rectangle for sampled region on last run through for loop
        if i == legCount
            yBounds = ylim;
            rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
            rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
            rectanglePos.height     = yBounds(2) - yBounds(1);
            rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
        end
        
        subplot(3,1,2)
        hold on
        plot(t, force.fullTrajectory.(EEselection)(1:length(t),2), lineColor{i}, 'DisplayName', EEselection);
        xlabel('time [s]')
        ylabel('force in y-direction [N]')
        grid on   
        %Plot rectangle for sampled region
        if i == legCount
            yBounds = ylim;
            rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
            rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
            rectanglePos.height     = yBounds(2) - yBounds(1);
            rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);            
        end
        
        subplot(3,1,3)
        hold on
        plot(t, force.fullTrajectory.(EEselection)(1:length(t),3), lineColor{i}, 'DisplayName', EEselection);
        xlabel('time [s]')
        ylabel('force in z-direction [N]')
        grid on    
        %Plot rectangle for sampled region  
        if i == legCount
            yBounds = ylim;
            rectanglePos.bottomLeft = [t(startIndexFullTrajectory), yBounds(1)];
            rectanglePos.width      =  t(endIndexFullTrajectory - startIndexFullTrajectory);
            rectanglePos.height     = yBounds(2) - yBounds(1);
            rectangle('Position', [rectanglePos.bottomLeft(1), rectanglePos.bottomLeft(2), rectanglePos.width, rectanglePos.height], 'LineWidth', 0.5);                
        end
    end
    if legCount > 1 % only display legend if there are multiple legs
        lgd = legend(p);
        lgd.FontSize = 14;
    end
    hold off
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
    %% Plot x vs z position of EE for the final trajectory (after trimming and averaging).
    figure('name', 'EE trajectory relative to hip', 'units','normalized','outerposition',outerPosition) 
    set(gcf,'color','w')
    % Number of rows [1 2], number of columns [1 2]
    subplotRows = ceil(legCount/2);
    if legCount < 2
        subplotColumns = 1;
    else
        subplotColumns = 2;
    end

    for i = 1:legCount
        EEselection = EEnames(i,:);
        subplot(subplotRows, subplotColumns,i)
        hold on
        % If we average the steps for cyclic motion also show the raw data
        % result
        if data.basicProperties.trajectory.averageStepsForCyclicalMotion
            plot(data.fullTrajectory.r.EEdes.(EEselection)(startIndexFullTrajectory:endIndexFullTrajectory,1), data.fullTrajectory.r.EEdes.(EEselection)(startIndexFullTrajectory:endIndexFullTrajectory,3), 'o', 'MarkerEdgeColor', '[0.5843 0.8157 0.9882]', 'MarkerFaceColor', '[ 0.5843 0.8157 0.9882]')
        end
        plot(data.(EEselection).r.EEdes(:,1), data.(EEselection).r.EEdes(:,3), 'k', 'LineWidth', 2)
        axis equal
        xlabel('x position [m]')
        ylabel('z position [m]')
        title(['End Effector Trajectory ', EEselection,])
        grid on
        hold off
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
end