%function [] = visualizeRobot(linkCount, robotProperties, Leg, meanCyclicMotionHipEE, EEselection, robotVisualization, dataExtraction, optimized, saveFiguresToPDF, fileName) 
 function visualizeRobot(results, classSelection, task, EEselection, fileName, robotVisualization, optimized) 

    EEnames     = results.(classSelection).(task).basicProperties.EEnames;
    jointNames  = results.(classSelection).(task).basicProperties.jointNames;
    legCount    = results.(classSelection).(task).basicProperties.legCount;
    linkCount    = results.(classSelection).(task).basicProperties.linkCount;
    robot       = results.(classSelection).(task).(EEselection).rigidBodyModel;
    robotProperties = results.(classSelection).(task).robotProperties;
    bodyLength  = robotProperties.baseLength;
    bodyWidth   = robotProperties.baseWidth;
    bodyHeight  = robotProperties.baseHeight;
    
    % Axis limits - adjust manually based on size of robot
    if robotVisualization.plotOneLeg
        xlimit = [-0.6, 0.6];
        ylimit = [-0.3, 0.3];
        zlimit = [-0.7, 0.2];
    else
        xlimit = [-1, 1];
        ylimit = [-0.65, 0.65];
        zlimit = [0, results.(classSelection).(task).base.position.LF(end,3)+0.5];
    end

    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        selectFrontHind = 1;
    else
        selectFrontHind = 2;
    end   
    
    finalPlottingIndex = length(results.(classSelection).(task).(EEselection).q) - 2;

    if robotVisualization.plotAllLegs
        finalPlottingIndex = [];
        for k = 1:legCount
            finalPlottingIndex = min([finalPlottingIndex, length(results.(classSelection).(task).(EEnames(k,:)).r.HAA)]);
        end
    end

    %% Get step positions for stairs
    inertialFrameEEPosition = results.(classSelection).(task).inertialFrame.EEpositionTrimmed;
    j = 0;
    stepPosition = [0 0 0];
    % Steps only plotted if motion is not averaged into cycles
    if ~results.(classSelection).(task).basicProperties.trajectory.averageStepsForCyclicalMotion
        for i = 1:finalPlottingIndex-1
            for k = 1:legCount
                if inertialFrameEEPosition.(EEnames(k,:))(i+1,3) == inertialFrameEEPosition.(EEnames(k,:))(i,3) && abs(inertialFrameEEPosition.(EEnames(k,:))(i,3)-inertialFrameEEPosition.(EEnames(k,:))(1,3)) > 0.03
                    j=j+1;
                    stepPosition(j,:) = [inertialFrameEEPosition.(EEnames(k,:))(i,1), inertialFrameEEPosition.(EEnames(k,:))(i,2), inertialFrameEEPosition.(EEnames(k,:))(i,3)];
                end
            end
        end

     if length(stepPosition(:,1))>1
     stepPosition = unique(stepPosition, 'rows');
        for i = 1:length(stepPosition(:,1))-1
            if abs(stepPosition(i,3) - stepPosition(i+1,3)) < 0.01
                stepPosition(i,:) = [0 0 0];
            end
        end
     end
        stepPosition = stepPosition(any(stepPosition,2),:);
        if length(stepPosition) > 1 && abs(stepPosition(1,3)) < 0.01
            stepPosition(1,:) = [];
        end
    end
    
    %% Get joint positions
    for i = 1:finalPlottingIndex
        if (linkCount == 2) 
            config(i,:) = [-results.(classSelection).(task).(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                           results.(classSelection).(task).(EEselection).q(i,1), ... % HAA
                           results.(classSelection).(task).(EEselection).q(i,2), ... % HFE
                           results.(classSelection).(task).(EEselection).q(i,3)]; % KFE
                       
        elseif (linkCount == 3)    
            config(i,:) = [-results.(classSelection).(task).(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                           results.(classSelection).(task).(EEselection).q(i,1), ... % HAA
                           results.(classSelection).(task).(EEselection).q(i,2), ... % HFE
                           results.(classSelection).(task).(EEselection).q(i,3), ... % KFE
                           results.(classSelection).(task).(EEselection).q(i,4)];    % AFE    
        elseif (linkCount == 4)
            config(i,:) = [-results.(classSelection).(task).(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                           results.(classSelection).(task).(EEselection).q(i,1), ... % HAA
                           results.(classSelection).(task).(EEselection).q(i,2), ... % HFE
                           results.(classSelection).(task).(EEselection).q(i,3), ... % KFE
                           results.(classSelection).(task).(EEselection).q(i,4), ... % AFE
                           results.(classSelection).(task).(EEselection).q(i,5)];    % DFE
        end
    end
    
    % Flat ground
    groundCoordinatesX = [2 2 -2 -2];
    groundCoordinatesY = [2 -2 -2 2];
    groundCoordinatesZ = -results.(classSelection).(task).base.position.(EEselection)(:,3)*[1 1 1 1];
  
    %% Display robot visualization for one leg
        % define patch shift which allows for body visualization
        if robotVisualization.plotOneLeg         
            if strcmp(EEselection, 'LF')
                patchShift = [0.05, 0, 0];
            elseif strcmp(EEselection, 'LH')
                patchShift = [-0.05+bodyLength, 0, 0];
            elseif strcmp(EEselection, 'RF')
                patchShift = [0.05, bodyWidth, 0];
            elseif strcmp(EEselection, 'RH')
                patchShift = [-0.05+bodyLength, bodyWidth, 0];
            end
        end

        % End effector displayed as sphere
        [xEE,yEE,zEE] = sphere;
        rEE = 0.02;
        xEE = rEE*xEE;
        yEE = rEE*yEE;
        zEE = rEE*zEE;
                      
        f1 = figure('units','normalized','outerposition',[0 0 1 1]); 
        set(gcf,'color','w')
        xlim([xlimit(1) xlimit(2)]);
        ylim([ylimit(1) ylimit(2)]);
        zlim([zlimit(1) zlimit(2)]);
        
        if robotVisualization.plotOneLeg      
        for j = 1:robotVisualization.numberOfStepsVisualized
            for i = 1:finalPlottingIndex
                
                % Set frames = on to see the coordinate system of the leg
                show(robot,config(i,:), 'Frames', 'off');
                xlim([xlimit(1) xlimit(2)]);
                ylim([ylimit(1) ylimit(2)]);
                zlim([zlimit(1) zlimit(2)]);
              
                hold on
                    if optimized && results.(classSelection).(task).basicProperties.optimizedLegs.(EEselection)
                        title(['Optimized ', EEselection])
                    else
                        title(['Nominal ', EEselection])
                    end
                % Define the vertices to show robot body
                vert = patchShift + ...
                       [0           0           -0.04;...
                       -bodyLength  0           -0.04;...
                       -bodyLength -bodyWidth   -0.04;...
                        0          -bodyWidth   -0.04;...
                        0           0            bodyHeight;...
                       -bodyLength  0            bodyHeight;...
                       -bodyLength -bodyWidth    bodyHeight;...
                        0          -bodyWidth    bodyHeight];

                % Compute body rotation about y axis with elementary rotation matrix
                bodyRotation = [cos(-config(i,1)), 0, sin(-config(i,1));
                                0                  1, 0;
                                -sin(-config(i,1)), 0 cos(-config(i,1))];

                % Apply body rotation to obtain new vertices
                vert = vert * bodyRotation;
                fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                patch('Vertices',vert,'Faces',fac,'FaceColor','w', 'FaceAlpha', 0.2)

                    % Ground color dependent on phase
                    if results.(classSelection).(task).(EEselection).force(i,3) > 0 
                        EEColor = 'gray'; % stance
                    else 
                        EEColor = 'white'; % swing
                    end 
                    
                    % Plot desired trajectory to observe tracking
                    plot3(results.(classSelection).(task).(EEselection).r.EEdes(1:end-2,1), ...
                          results.(classSelection).(task).(EEselection).r.EEdes(1:end-2,2), ...
                          results.(classSelection).(task).(EEselection).r.EEdes(1:end-2,3),'k', 'LineWidth', 1)                    
                     
                      % Plot end effector sphere
                        surf(xEE+results.(classSelection).(task).(EEselection).r.EE(i,1),yEE+results.(classSelection).(task).(EEselection).r.EE(i,2),zEE+results.(classSelection).(task).(EEselection).r.EE(i,3), 'edgecolor','none')
                        colormap(EEColor);

                        % Plot links as cylinders

                        if optimized && results.(classSelection).(task).basicProperties.optimizedLegs.(EEselection) % If the selected leg has been optimized, use optimized link lengths
                            rHAA = [results.(classSelection).(task).(EEselection).rOpt.HAA(i,1) results.(classSelection).(task).(EEselection).rOpt.HAA(i,2) results.(classSelection).(task).(EEselection).rOpt.HAA(i,3)];
                            rHFE = [results.(classSelection).(task).(EEselection).rOpt.HFE(i,1) results.(classSelection).(task).(EEselection).rOpt.HFE(i,2) results.(classSelection).(task).(EEselection).rOpt.HFE(i,3)];
                            rKFE = [results.(classSelection).(task).(EEselection).rOpt.KFE(i,1) results.(classSelection).(task).(EEselection).rOpt.KFE(i,2) results.(classSelection).(task).(EEselection).rOpt.KFE(i,3)];
                            rAFE = [results.(classSelection).(task).(EEselection).rOpt.AFE(i,1) results.(classSelection).(task).(EEselection).rOpt.AFE(i,2) results.(classSelection).(task).(EEselection).rOpt.AFE(i,3)];
                            rDFE = [results.(classSelection).(task).(EEselection).rOpt.DFE(i,1) results.(classSelection).(task).(EEselection).rOpt.DFE(i,2) results.(classSelection).(task).(EEselection).rOpt.DFE(i,3)];
                            rEE  = [results.(classSelection).(task).(EEselection).r.EE(i,1) results.(classSelection).(task).(EEselection).r.EE(i,2) results.(classSelection).(task).(EEselection).r.EE(i,3)];
                        else
                            rHAA = [results.(classSelection).(task).(EEselection).r.HAA(i,1) results.(classSelection).(task).(EEselection).r.HAA(i,2) results.(classSelection).(task).(EEselection).r.HAA(i,3)];
                            rHFE = [results.(classSelection).(task).(EEselection).r.HFE(i,1) results.(classSelection).(task).(EEselection).r.HFE(i,2) results.(classSelection).(task).(EEselection).r.HFE(i,3)];
                            rKFE = [results.(classSelection).(task).(EEselection).r.KFE(i,1) results.(classSelection).(task).(EEselection).r.KFE(i,2) results.(classSelection).(task).(EEselection).r.KFE(i,3)];
                            rAFE = [results.(classSelection).(task).(EEselection).r.AFE(i,1) results.(classSelection).(task).(EEselection).r.AFE(i,2) results.(classSelection).(task).(EEselection).r.AFE(i,3)];
                            rDFE = [results.(classSelection).(task).(EEselection).r.DFE(i,1) results.(classSelection).(task).(EEselection).r.DFE(i,2) results.(classSelection).(task).(EEselection).r.DFE(i,3)];
                            rEE  = [results.(classSelection).(task).(EEselection).r.EE(i,1) results.(classSelection).(task).(EEselection).r.EE(i,2) results.(classSelection).(task).(EEselection).r.EE(i,3)];
                        end
                        
                        [x1,y1,z1] = cylinder2P(robotProperties.hip(selectFrontHind).radius, 20, rHAA,rHFE);
                        [x2,y2,z2] = cylinder2P(robotProperties.thigh(selectFrontHind).radius, 20,rHFE,rKFE);
                        [x3,y3,z3] = cylinder2P(robotProperties.shank(selectFrontHind).radius, 20,rKFE,rAFE);
                        surf(x1, y1, z1, 'edgecolor','none')
                        surf(x2, y2, z2, 'edgecolor','none')
                        surf(x3, y3, z3, 'edgecolor','none')   
                        
                        if linkCount > 2 
                            [x4,y4,z4] = cylinder2P(robotProperties.foot(selectFrontHind).radius, 20,rAFE,rDFE);
                            surf(x4, y4, z4, 'edgecolor','none')
                        end
                        
                        if linkCount == 4
                            [x5,y5,z5] = cylinder2P(robotProperties.phalanges(selectFrontHind).radius, 20,rDFE,rEE);
                            surf(x5, y5, z5, 'edgecolor','none')
                        end
                        
                        % Save to gif
                        if mod(i,2) % Save every 2nd frame into gif
                            % Capture the plot as an image 
                            frame = getframe(f1); 
                            im = frame2im(frame); 
                            [imind,cm] = rgb2ind(im,256); 
                            % Write to the GIF File 
                            if i == 1 && strcmp(EEselection, 'LF')
                                imwrite(imind,cm,fileName,'gif', 'Loopcount',inf); 
                            else 
                                imwrite(imind,cm, fileName,'gif','WriteMode','append'); 
                            end 
                        end  
                hold off
            end
        end
   end
        
  %% Plot all legs
  viewMultipleAngles = false;
  if viewMultipleAngles
      numberOfViewWindows = 3;
  else
      numberOfViewWindows = 1;
  end
  
    if robotVisualization.plotAllLegs 
        for i = 1:finalPlottingIndex
            figure(f1); % iso view
            for viewIndex = 1:numberOfViewWindows
            if viewMultipleAngles
                if viewIndex == 1 % Iso view
                    subplot(2,2,[2 4])
                    show(robot, config(i,:), 'Frames', 'off');
                    view([-30 10])
                    xlim([xlimit(1) xlimit(2)]);
                    ylim([ylimit(1) ylimit(2)]);
                    zlim([zlimit(1) zlimit(2)]);
                elseif viewIndex == 2 % Top View
                    subplot(2,2,1)
                    show(robot, config(i,:), 'Frames', 'off');
                    view([0 90])
                    xlim([xlimit(1) xlimit(2)]);
                    ylim([ylimit(1) ylimit(2)]);
                    zlim([zlimit(1) zlimit(2)]);
                else % Side view
                    subplot(2,2,3)
                    show(robot, config(i,:), 'Frames', 'off');
                    view([0 0])
                    xlim([xlimit(1) xlimit(2)]);
                    ylim([ylimit(1) ylimit(2)]);
                    zlim([0 zlimit(2)]);
                end
            else
                show(robot, config(i,:), 'Frames', 'off');
                view([-30 10])
                xlim([xlimit(1) xlimit(2)]);
                ylim([ylimit(1) ylimit(2)]);
                zlim([zlimit(1) zlimit(2)]);         
            end
            
            hold on
            
            % Plot moving lines on ground to show robot movement speed
            groundGridlines = linspace(-1,40,411) - results.(classSelection).(task).base.position.(EEselection)(i,1);
            plot([groundGridlines(:),groundGridlines(:)],[-3,3], 'k')

                %% Plot all legs  
                if legCount > 2
                    xNom.LF = robotProperties.xNom(1); yNom.LF = robotProperties.yNom(1); zNom.LF = robotProperties.zNom;
                    xNom.RF = robotProperties.xNom(1); yNom.RF = -robotProperties.yNom(1); zNom.RF = robotProperties.zNom;
                    xNom.LH = -robotProperties.xNom(2); yNom.LH = robotProperties.yNom(2); zNom.LH = robotProperties.zNom;
                    xNom.RH = -robotProperties.xNom(2); yNom.RH = -robotProperties.yNom(2); zNom.RH = robotProperties.zNom;
                else
                    xNom.LF = robotProperties.xNom(1); yNom.LF = robotProperties.yNom(1); zNom.LF = robotProperties.zNom(1);
                    xNom.RF = robotProperties.xNom(1); yNom.RF = -robotProperties.yNom(1); zNom.RF = robotProperties.zNom(1);
                end
                
                % Apply body rotation to robot nominal positions so 
                % multiple legs shown in correct position. Not
                % necessary if plotting only one leg because the hip
                % attachment frame is at origin.
                for k = 1:legCount
                    if strcmp(EEnames(k,:), 'LH') || strcmp(EEnames(k,:), 'RH')
                        translationDirection = -1;
                    else
                        translationDirection = 1;
                    end
                    xNom.(EEnames(k,:)) = xNom.(EEnames(k,:))*(cos(-config(i,1)));
                    zNom.(EEnames(k,:)) = zNom.(EEnames(k,:)) + translationDirection*xNom.LF*(sin(-config(i,1)));
                end
                
                bodyCenterX = [];
                for k = 1:legCount
                    bodyCenterX = [bodyCenterX, xNom.(EEnames(k,:))];
                end
                bodyCenterX = mean(bodyCenterX);
                
                     vert = [-bodyCenterX/2 0 0] + ...
                            [0.5*bodyLength,  0.5*bodyWidth, -0.04;...
                            -0.5*bodyLength,  0.5*bodyWidth, -0.04;...
                            -0.5*bodyLength, -0.5*bodyWidth, -0.04;...
                             0.5*bodyLength, -0.5*bodyWidth, -0.04;...
                             0.5*bodyLength,  0.5*bodyWidth, bodyHeight;...
                            -0.5*bodyLength,  0.5*bodyWidth, bodyHeight;...
                            -0.5*bodyLength, -0.5*bodyWidth, bodyHeight;...
                             0.5*bodyLength, -0.5*bodyWidth, bodyHeight];

                    % Compute body rotation about y axis with elementary rotation matrix
                    bodyRotation = [cos(-config(i,1)), 0, sin(-config(i,1));
                                    0                  1, 0;
                                   -sin(-config(i,1)), 0  cos(-config(i,1))];

                    % Apply body rotation to obtain new vertices
                    vert = vert*bodyRotation + [0,0,-groundCoordinatesZ(i,1)];
                    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                   
                    % Plot body patch
                    patch('Vertices',vert,'Faces',fac,'FaceColor','w', 'FaceAlpha', 0.2)    

                      % Get ground color dependent on phase
                        if legCount > 1
                            if results.(classSelection).(task).LF.force(i,3) > 0 && results.(classSelection).(task).RF.force(i,3) > 0 % Trot, all legs stance
                                groundColor = 'c';
                            elseif (results.(classSelection).(task).LF.force(i,3) > 0 && results.(classSelection).(task).RF.force(i,3) == 0) % Trot, LF/LH stance
                                groundColor = 'c';
                            else % Trot, LF/LH stance
                                groundColor = 'c'; 
                            end
                        else
                            if results.(classSelection).(task).LF.force(i,3) > 0
                                groundColor = 'c';
                            else
                                groundColor = 'c';
                            end
                        end
                        
                        %% Plot ground and stairs
                        patch(groundCoordinatesX(1,:), groundCoordinatesY(1,:), [0 0 0 0], groundColor, 'FaceAlpha', 0.5)
                        
                        for k = 1:legCount
                            if results.(classSelection).(task).(EEnames(k,:)).force(i,3) > 0
                                groundColorStairs.(EEnames(k,:)) = 'c';
                            else
                                groundColorStairs.(EEnames(k,:)) = 'c';
                            end
  
                            for n = 1:length(stepPosition(:,1))
                                patch((stepPosition(n,1)-results.(classSelection).(task).base.position.(EEselection)(i,1))+0.05*[5 5 -3 -3], stepPosition(n,2) + 3*[1 -1 -1 1], stepPosition(n,3)*[1 1 1 1], groundColorStairs.(EEnames(k,:)), 'FaceAlpha', 0.5)
                            end
                        end
         
                    %% Display links and end effectors
                    for k = 1:legCount
                        EEselection = EEnames(k,:);
                        
                        % Display end effectors as spheres
                        surf(xNom.(EEselection)+xEE+results.(classSelection).(task).(EEselection).r.EE(i,1),yNom.(EEselection)+yEE+results.(classSelection).(task).(EEselection).r.EE(i,2),zNom.(EEselection)+zEE+results.(classSelection).(task).(EEselection).r.EE(i,3)-groundCoordinatesZ(i,1), 'edgecolor','none')
                        colormap('white');

                          % Display links as cylinders
                        if ~optimized || (optimized && ~results.(classSelection).(task).basicProperties.optimizedLegs.(EEselection))
                            rHAA = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HAA(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HAA(i,2), zNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HAA(i,3)];
                            rHFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HFE(i,2), zNom.(EEselection)+results.(classSelection).(task).(EEselection).r.HFE(i,3)];
                            rKFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.KFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.KFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).r.KFE(i,3)];
                            rAFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.AFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.AFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).r.AFE(i,3)];
                            rDFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.DFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.DFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).r.DFE(i,3)];
                            rEE  = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).r.EE(i,1),  yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.EE(i,2),  zNom.(EEselection)+results.(classSelection).(task).(EEselection).r.EE(i,3)];
                        end
                        if optimized && results.(classSelection).(task).basicProperties.optimizedLegs.(EEselection) % If the selected leg has been optimized, use optimized link lengths
                            rHAA = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.HAA(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.HAA(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).rOpt.HAA(i,3)];
                            rHFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.HFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.HFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).rOpt.HFE(i,3)];
                            rKFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.KFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.KFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).rOpt.KFE(i,3)];
                            rAFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.AFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.AFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).rOpt.AFE(i,3)];
                            rDFE = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.DFE(i,1), yNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.DFE(i,2), zNom.(EEselection)+ results.(classSelection).(task).(EEselection).rOpt.DFE(i,3)];
                            rEE  = [xNom.(EEselection)+results.(classSelection).(task).(EEselection).rOpt.EE(i,1),  yNom.(EEselection)+results.(classSelection).(task).(EEselection).r.EE(i,2),     zNom.(EEselection)+ results.(classSelection).(task).(EEselection).r.EE(i,3)];
                        end
                        
                        [x1,y1,z1] = cylinder2P(robotProperties.hip(selectFrontHind).radius, 10, rHAA,rHFE);
                        [x2,y2,z2] = cylinder2P(robotProperties.thigh(selectFrontHind).radius, 10,rHFE,rKFE);
                        [x3,y3,z3] = cylinder2P(robotProperties.shank(selectFrontHind).radius, 10,rKFE,rAFE);
                        surf(x1, y1, z1- groundCoordinatesZ(i,1), 'edgecolor','none')
                        surf(x2, y2, z2- groundCoordinatesZ(i,1), 'edgecolor','none')
                        surf(x3, y3, z3- groundCoordinatesZ(i,1), 'edgecolor','none')   
                        
                        if linkCount > 2 
                            [x4,y4,z4] = cylinder2P(robotProperties.foot(selectFrontHind).radius, 10,rAFE,rDFE);
                            surf(x4, y4, z4- groundCoordinatesZ(i,1), 'edgecolor','none')
                        end
                        
                        if linkCount == 4
                            [x5,y5,z5] = cylinder2P(robotProperties.phalanges(selectFrontHind).radius, 10,rDFE,rEE);
                            surf(x5, y5, z5-groundCoordinatesZ(i,1), 'edgecolor','none')
                        end
                    end
                    hold off
                end
                hold off 

            if mod(i,3)==0 || i==1% Save every 3rd frame into gif
                  % Capture f1 as image and save to .gif
                  frame = getframe(f1); 
                  im = frame2im(frame); 
                  [imind,cm] = rgb2ind(im,256);
                  if i == 1 
                      imwrite(imind,cm,fileName,'gif', 'Loopcount',inf); 
                  else 
                      imwrite(imind,cm, fileName,'gif','WriteMode','append'); 
                  end 
            end  
        end
    end
end
