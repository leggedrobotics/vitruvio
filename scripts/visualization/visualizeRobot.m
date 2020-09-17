 function visualizeRobot(data, EEselection, fileName, robotVisualization, optimized) 

    EEnames         = data.basicProperties.EEnames;
    jointNames      = data.basicProperties.jointNames;
    legCount        = data.basicProperties.legCount;
    linkCount       = data.basicProperties.linkCount;
    classSelection  = data.basicProperties.classSelection;
    task            = data.basicProperties.task;
    robotProperties = data.robotProperties;
    bodyLength      = robotProperties.baseLength;
    bodyWidth       = robotProperties.baseWidth;
    bodyHeight      = robotProperties.baseHeight;
    displayTorso    = robotVisualization.torso;
    
    createGif = robotVisualization.createGif;
    
    %outerPosition = [0 0 1 1]; % Fullscreen
    outerPosition = [0.5 0.5 0.5 0.5]; % Top right corner
    
    if optimized && data.basicProperties.optimizedLegs.(EEselection)
        robot = data.(EEselection).rigidBodyModelSwingOpt;
    else
        robot = data.(EEselection).rigidBodyModelSwing;
    end
    
    % Specify dimensions of torso if applicable
    torsoHeight = 0.4;
    torsoLength = 0.22;
    torsoWidth  = 0.28;
    
    groundViz    = [];
    gridlinesViz = [];
    torsoViz     = [];
    bodyViz      = [];
    platformViz  = [];
    stairViz     = [];
    
    groundGridlines = linspace(-1,99,101);
    platformCoordinates = 0;

    myMap  = [65 105 225]/225; % Royal blue
    
    for i = 1:legCount
        hipViz.(EEnames(i,:))       = [];
        thighViz.(EEnames(i,:))     = [];
        shankViz.(EEnames(i,:))     = [];
        footViz.(EEnames(i,:))      = [];
        phalangesViz.(EEnames(i,:)) = [];
        EEviz.(EEnames(i,:))        = [];
    end
    
    % Axis limits - adjust manually based on size of robot
    if robotVisualization.oneLeg
        xlimit = [-1, 1];
        ylimit = [-0.7, 0.7];
        zlimit = [-1.1, 1];
    else
        xlimit = [-1.05, 1.05];
        ylimit = [-1, 1];
        zlimit = [0, data.base.position.LF(end,3)+1];
        if displayTorso
            zlimit(2) = zlimit(2) + 0.4;
        end
    end

    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        selectFrontHind = 1;
    else
        selectFrontHind = 2;
    end   
    
    finalPlottingIndex = length(data.(EEselection).q) - 2;

    if robotVisualization.allLegs
        finalPlottingIndex = [];
        for k = 1:legCount
            finalPlottingIndex = min([finalPlottingIndex, length(data.(EEnames(k,:)).r.HAA)]);
        end
    end

    %% Get step positions for stairs
    inertialFrameEEPosition = data.inertialFrame.EEpositionTrimmed;
    j = 0;
    stepPosition = [0 0 0];
    % Steps (stairs) only plotted if motion is not averaged into cycles
    if ~data.basicProperties.trajectory.averageStepsForCyclicalMotion
        for i = 1:finalPlottingIndex-1
            for k = 1:legCount
                if inertialFrameEEPosition.(EEnames(k,:))(i+1,3) == inertialFrameEEPosition.(EEnames(k,:))(i,3) && abs(inertialFrameEEPosition.(EEnames(k,:))(i,3)-inertialFrameEEPosition.(EEnames(k,:))(1,3)) > 0.03
                    j=j+1;
                    stepPosition(j,:) = [inertialFrameEEPosition.(EEnames(k,:))(i,1), inertialFrameEEPosition.(EEnames(k,:))(i,2), inertialFrameEEPosition.(EEnames(k,:))(i,3)];
                end
            end
        end
        
        if ~isequal(stepPosition, [0 0 0])
            while any(abs(stepPosition(1:end-1,3)-stepPosition(2:end,3))<0.01)
                 if length(stepPosition(:,1))>1
                 stepPosition = unique(stepPosition, 'rows');
                    for i = 1:length(stepPosition(:,1))-1
                        if abs(stepPosition(i+1,3) - stepPosition(i,3)) < 0.01
                            stepPosition(i+1,:) = [0 0 0];
                        end
                    end
                 end

                stepPosition = stepPosition(any(stepPosition,2),:);
                if length(stepPosition) > 1 && abs(stepPosition(1,3)) < 0.01
                    stepPosition(1,:) = [];
                end
            end
        end
        
        % Final platform
        platformCoordinates = 0;
        if abs(inertialFrameEEPosition.(EEnames(k,:))(end,3)-inertialFrameEEPosition.(EEnames(k,:))(1,3))>0.03
            if ~isempty(stepPosition)
                platformStartPosition = stepPosition(end,1);
                % Find last time EE was in contact and take the ground height
                % at this time
                footInContact = data.LF.force(:,3);
                lastContactIndex = find(abs(footInContact==0),1,'last');
                platformHeight = inertialFrameEEPosition.LF(lastContactIndex,3);
                platformCoordinates = [platformStartPosition, inertialFrameEEPosition.LF(end,2)-0.5*bodyWidth, platformHeight];
                platformLength =  inertialFrameEEPosition.LF(lastContactIndex,1) - platformStartPosition + 0.5;
            end
        end
    end
    
    %% Get joint positions
    for i = 1:finalPlottingIndex
        if optimized && data.basicProperties.optimizedLegs.(EEselection)
            if (linkCount == 2) 
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).qOpt(i,1), ... % HAA
                                              data.(EEselection).qOpt(i,2), ... % HFE
                                              data.(EEselection).qOpt(i,3)];    % KFE

            elseif (linkCount == 3)    
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).qOpt(i,1), ... % HAA
                                              data.(EEselection).qOpt(i,2), ... % HFE
                                              data.(EEselection).qOpt(i,3), ... % KFE
                                              data.(EEselection).qOpt(i,4)];    % AFE    
            elseif (linkCount == 4)
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).qOpt(i,1), ... % HAA
                                              data.(EEselection).qOpt(i,2), ... % HFE
                                              data.(EEselection).qOpt(i,3), ... % KFE
                                              data.(EEselection).qOpt(i,4), ... % AFE
                                              data.(EEselection).qOpt(i,5)];    % DFE
            end
        else
            if (linkCount == 2) 
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).q(i,1), ... % HAA
                                              data.(EEselection).q(i,2), ... % HFE
                                              data.(EEselection).q(i,3)];    % KFE

            elseif (linkCount == 3)    
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).q(i,1), ... % HAA
                                              data.(EEselection).q(i,2), ... % HFE
                                              data.(EEselection).q(i,3), ... % KFE
                                              data.(EEselection).q(i,4)];    % AFE    
            elseif (linkCount == 4)
                config.(EEselection)(i,:) = [data.(EEselection).body.eulerAngles(i,2), ... %body rotation about inertial y
                                              data.(EEselection).q(i,1), ... % HAA
                                              data.(EEselection).q(i,2), ... % HFE
                                              data.(EEselection).q(i,3), ... % KFE
                                              data.(EEselection).q(i,4), ... % AFE
                                              data.(EEselection).q(i,5)];    % DFE
            end
        end
    end
    
    % Flat ground
    groundCoordinatesX = [2 2 -2 -2];
    groundCoordinatesY = [2 -2 -2 2];
    groundCoordinatesZ = -data.base.position.(EEselection)(:,3)*[1 1 1 1];
  
    %% Visualization for one leg - used to visualize with rigid body tree
        % define patch shift which allows for body visualization
        if robotVisualization.oneLeg         
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
                      
        f1 = figure('name', 'Visualization','units','normalized','outerposition',outerPosition); 
        set(gcf,'color','w')
        xlim([xlimit(1) xlimit(2)]);
        ylim([ylimit(1) ylimit(2)]);
        zlim([zlimit(1) zlimit(2)]);
        
    if robotVisualization.oneLeg  
        for i = 1:finalPlottingIndex
            hold off
            figure(f1);
            % Set frames = on to see the rigid body tree of the leg
            show(robot,config.(EEselection)(i,:), 'Frames', 'off');
            grid on
            xlim([xlimit(1) xlimit(2)]);
            ylim([ylimit(1) ylimit(2)]);
            zlim([zlimit(1) zlimit(2)]);
            hold on
                if optimized && data.basicProperties.optimizedLegs.(EEselection)
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

            % Compute body rotation about y axis (pitch) with elementary rotation matrix
            bodyRotation = [cos(-config.(EEselection)(i,1)), 0, sin(-config.(EEselection)(i,1));
                            0                  1, 0;
                            -sin(-config.(EEselection)(i,1)), 0 cos(-config.(EEselection)(i,1))];

            % Apply body rotation to obtain new vertices
            vert = vert * bodyRotation;
            fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
            patch('Vertices', vert, 'Faces', fac, 'FaceColor', 'w', 'FaceAlpha', 0.2)

            % Plot desired trajectory to observe tracking
            plot3(data.(EEselection).r.EEdes(1:end-2,1), ...
                  data.(EEselection).r.EEdes(1:end-2,2), ...
                  data.(EEselection).r.EEdes(1:end-2,3),'k', 'LineWidth', 1)                    

            % Plot end effector sphere
            surf(xEE+data.(EEselection).r.EE(i,1), yEE+data.(EEselection).r.EE(i,2), zEE+data.(EEselection).r.EE(i,3), 'edgecolor', 'none')
            colormap(myMap)

            % Plot links as cylinders
            if optimized && data.basicProperties.optimizedLegs.(EEselection) % If the selected leg has been optimized, use optimized link lengths
                rHAA = [data.(EEselection).rOpt.HAA(i,1) data.(EEselection).rOpt.HAA(i,2) data.(EEselection).rOpt.HAA(i,3)];
                rHFE = [data.(EEselection).rOpt.HFE(i,1) data.(EEselection).rOpt.HFE(i,2) data.(EEselection).rOpt.HFE(i,3)];
                rKFE = [data.(EEselection).rOpt.KFE(i,1) data.(EEselection).rOpt.KFE(i,2) data.(EEselection).rOpt.KFE(i,3)];
                rAFE = [data.(EEselection).rOpt.AFE(i,1) data.(EEselection).rOpt.AFE(i,2) data.(EEselection).rOpt.AFE(i,3)];
                rDFE = [data.(EEselection).rOpt.DFE(i,1) data.(EEselection).rOpt.DFE(i,2) data.(EEselection).rOpt.DFE(i,3)];
                rEE  = [data.(EEselection).r.EE(i,1) data.(EEselection).r.EE(i,2) data.(EEselection).r.EE(i,3)];
            else
                rHAA = [data.(EEselection).r.HAA(i,1) data.(EEselection).r.HAA(i,2) data.(EEselection).r.HAA(i,3)];
                rHFE = [data.(EEselection).r.HFE(i,1) data.(EEselection).r.HFE(i,2) data.(EEselection).r.HFE(i,3)];
                rKFE = [data.(EEselection).r.KFE(i,1) data.(EEselection).r.KFE(i,2) data.(EEselection).r.KFE(i,3)];
                rAFE = [data.(EEselection).r.AFE(i,1) data.(EEselection).r.AFE(i,2) data.(EEselection).r.AFE(i,3)];
                rDFE = [data.(EEselection).r.DFE(i,1) data.(EEselection).r.DFE(i,2) data.(EEselection).r.DFE(i,3)];
                rEE  = [data.(EEselection).r.EE(i,1) data.(EEselection).r.EE(i,2) data.(EEselection).r.EE(i,3)];
            end

            [x1,y1,z1] = cylinder2P(robotProperties.hip(selectFrontHind).radius, 40, rHAA,rHFE);
            [x2,y2,z2] = cylinder2P(robotProperties.thigh(selectFrontHind).radius, 40,rHFE,rKFE);
            [x3,y3,z3] = cylinder2P(robotProperties.shank(selectFrontHind).radius, 40,rKFE,rAFE);
            surf(x1, y1, z1, 'edgecolor','none')
            surf(x2, y2, z2, 'edgecolor','none')
            surf(x3, y3, z3, 'edgecolor','none')   

            if linkCount > 2 
                [x4,y4,z4] = cylinder2P(robotProperties.foot(selectFrontHind).radius, 40,rAFE,rDFE);
                surf(x4, y4, z4, 'edgecolor','none')
            end

            if linkCount == 4
                [x5,y5,z5] = cylinder2P(robotProperties.phalanges(selectFrontHind).radius, 40,rDFE,rEE);
                surf(x5, y5, z5, 'edgecolor','none')
            end
            
            hold off
            % Save to gif
            if createGif
                if mod(i,2) % Save every 2nd frame into gif
                    % Capture the plot as an image 
                    frame = getframe(f1); 
                    im = frame2im(frame); 
                    [imind,cm] = rgb2ind(im,256); 
                    % Write to the GIF File 
                    if i == 1 && isequal(EEselection, 'LF')
                        imwrite(imind,cm,fileName,'gif', 'Loopcount',inf); 
                    else 
                        imwrite(imind,cm, fileName,'gif','WriteMode','append'); 
                    end 
                end  
            end
        end
   end
        
  %% Visualization for all legs
    if robotVisualization.allLegs 
        for i = 1:finalPlottingIndex
            hold off
            figure(f1);
            grid off
            hold off
            %view([0 0]) % Side view
            view([-20 5]) % Sets view angle for 3D
            xlim([xlimit(1) xlimit(2)]);
            ylim([ylimit(1) ylimit(2)]);
            zlim([zlimit(1) zlimit(2)]);         
           
            set(gca,'visible','off')
            set(gca,'xtick',[])
            hold on
            
            % Plot moving lines on ground to show robot movement speed
            delete(gridlinesViz);
            gridlinesViz = plot([groundGridlines(:)-data.base.position.(EEselection)(i,1),groundGridlines(:)-data.base.position.(EEselection)(i,1)],[-3,3], 'k', 'LineWidth', 0.5);

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
                    xNom.(EEnames(k,:)) = xNom.(EEnames(k,:))*(cos(-config.(EEselection)(i,1)));
                    zNom.(EEnames(k,:)) = zNom.(EEnames(k,:)) + xNom.(EEnames(k,:))*(sin(-config.(EEselection)(i,1)));
                end
                
                bodyCenterX = [];
                bodyCenterZ = [];
                for k = 1:legCount
                    bodyCenterX = [bodyCenterX, xNom.(EEnames(k,:))];
                    bodyCenterZ = [bodyCenterZ, zNom.(EEnames(k,:))];
                end
                bodyCenterX = mean(bodyCenterX);
                bodyCenterZ = mean(bodyCenterZ);
                
                 vert = [bodyCenterX,     0,             bodyCenterZ] + ...
                        [0.5*bodyLength,  0.5*bodyWidth, 0;...
                        -0.5*bodyLength,  0.5*bodyWidth, 0;...
                        -0.5*bodyLength, -0.5*bodyWidth, 0;...
                         0.5*bodyLength, -0.5*bodyWidth, 0;...
                         0.5*bodyLength,  0.5*bodyWidth, bodyHeight;...
                        -0.5*bodyLength,  0.5*bodyWidth, bodyHeight;...
                        -0.5*bodyLength, -0.5*bodyWidth, bodyHeight;...
                         0.5*bodyLength, -0.5*bodyWidth, bodyHeight];

                    vertTorso = vert;    
                    % Compute body rotation about y axis with elementary rotation matrix
                    bodyRotation = [cos(-config.(EEselection)(i,1)), 0, sin(-config.(EEselection)(i,1));
                                    0                  1, 0;
                                   -sin(-config.(EEselection)(i,1)), 0  cos(-config.(EEselection)(i,1))];

                    % Apply body rotation to obtain new vertices
                    vert = vert*bodyRotation + [0,0,-groundCoordinatesZ(i,1)];
                    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                   
                    % Plot body patch
                    delete(bodyViz);
                    bodyViz = patch('Vertices',vert,'Faces',fac,'FaceColor','w', 'FaceAlpha', 0.7);    

                    %% Torso for centaur robot
                    if displayTorso
                        vertTorsoWidth = vertTorso(:,2);
                        vertTorsoWidth(vertTorsoWidth<0) = -0.5*torsoWidth;
                        vertTorsoWidth(vertTorsoWidth>0) = 0.5*torsoWidth;
                        vertTorso(:,2) = vertTorsoWidth;
                        
                        vertTorso(1:4,3) = vertTorso(5:end,3); % shift torso up
                        vertTorso(5:end,3) = vertTorso(5:end,3) + torsoHeight; % Extend torso height
                        
                        % Shift back of torso forward
                        vertTorso(2,1) = vertTorso(1,1) - torsoLength;
                        vertTorso(3,1) = vertTorso(1,1) - torsoLength;
                        vertTorso(6,1) = vertTorso(1,1) - torsoLength;
                        vertTorso(7,1) = vertTorso(1,1) - torsoLength;
                        
                        % Apply body rotation
                        vertTorso = vertTorso*bodyRotation + [0,0,-groundCoordinatesZ(i,1)];
                        delete(torsoViz);
                        torsoViz = patch('Vertices',vertTorso,'Faces',fac,'FaceColor','w', 'FaceAlpha', 0.7);    
                    end
                    
                        %% Plot ground and stairs
                        groundColor = myMap;
                        groundColorStairs =  myMap;
                        delete(groundViz)
                        groundViz = patch(groundCoordinatesX(1,:), groundCoordinatesY(1,:), [0 0 0 0], groundColor, 'FaceAlpha', 0.4, 'LineStyle', '-', 'LineWidth', 0.5);
                        delete(stairViz);
                        if ~isequal(stepPosition, [0 0 0])
                            for n = 1:length(stepPosition(:,1))
                                stairViz(n) = patch(stepPosition(n,1)-data.base.position.LF(i,1)+0.05*[5 5 -1 -1], stepPosition(n,2) + 3*[1 -1 -1 1], stepPosition(n,3)*[1 1 1 1], groundColorStairs, 'FaceAlpha', 0.4, 'LineStyle', '-', 'LineWidth', 0.5);
                            end
                        end
                        
                        % Plot final platform
                        if platformCoordinates ~= 0
                            delete(platformViz)
                            platformViz = patch(platformCoordinates(1,1)-data.base.position.LF(i,1)+platformLength*[1 1 0 0], platformCoordinates(1,2)+2*[1 -1 -1 1], platformCoordinates(1,3)*[1 1 1 1], groundColorStairs, 'FaceAlpha', 0.4, 'LineStyle', 'none');
                        end
                    %% Display links and end effectors
                    edgeColor = 'k';

                    for k = 1:legCount
                        delete(EEviz.(EEnames(k,:)));
                        delete(hipViz.(EEnames(k,:)));
                        delete(thighViz.(EEnames(k,:)));
                        delete(shankViz.(EEnames(k,:)));
                        delete(footViz.(EEnames(k,:)));
                        delete(phalangesViz.(EEnames(k,:)));
                   
                        % Display links as cylinders
                        if ~optimized || (optimized && ~data.basicProperties.optimizedLegs.(EEnames(k,:)))
                            rHAA = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HAA(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HAA(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HAA(i,3)];
                            rHFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.HFE(i,3)];
                            rKFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.KFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.KFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.KFE(i,3)];
                            rAFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.AFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.AFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.AFE(i,3)];
                            rDFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.DFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.DFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.DFE(i,3)];
                            rEE  = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.EE(i,1),  yNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.EE(i,2),  zNom.(EEnames(k,:)) + data.(EEnames(k,:)).r.EE(i,3)];
                        end
                        if optimized && data.basicProperties.optimizedLegs.(EEnames(k,:)) % If the selected leg has been optimized, use optimized link lengths
                            rHAA = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HAA(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HAA(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HAA(i,3)];
                            rHFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.HFE(i,3)];
                            rKFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.KFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.KFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.KFE(i,3)];
                            rAFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.AFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.AFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.AFE(i,3)];
                            rDFE = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.DFE(i,1), yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.DFE(i,2), zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.DFE(i,3)];
                            rEE  = [xNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.EE(i,1),  yNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.EE(i,2),  zNom.(EEnames(k,:)) + data.(EEnames(k,:)).rOpt.EE(i,3)];
                        end
                        
                        [x1,y1,z1] = cylinder2P(robotProperties.hip(selectFrontHind).radius, 8, rHAA, rHFE);
                        [x2,y2,z2] = cylinder2P(robotProperties.thigh(selectFrontHind).radius, 8,rHFE, rKFE);
                        [x3,y3,z3] = cylinder2P(robotProperties.shank(selectFrontHind).radius, 8,rKFE, rAFE);
                        hipViz.(EEnames(k,:))   = surf(x1, y1, z1- groundCoordinatesZ(i,1), 'edgecolor', edgeColor, 'LineWidth', 0.01);
                        thighViz.(EEnames(k,:)) = surf(x2, y2, z2- groundCoordinatesZ(i,1), 'edgecolor', edgeColor, 'LineWidth', 0.01);
                        shankViz.(EEnames(k,:)) = surf(x3, y3, z3- groundCoordinatesZ(i,1), 'edgecolor', edgeColor, 'LineWidth', 0.01);   
                        
                        if linkCount > 2 
                            [x4,y4,z4] = cylinder2P(robotProperties.foot(selectFrontHind).radius, 8, rAFE, rDFE);
                            footViz.(EEnames(k,:)) = surf(x4, y4, z4- groundCoordinatesZ(i,1), 'edgecolor', edgeColor, 'LineWidth', 0.1);
                        end
                        
                        if linkCount == 4
                            [x5,y5,z5] = cylinder2P(robotProperties.phalanges(selectFrontHind).radius, 8, rDFE, rEE);
                            phalangesViz.(EEnames(k,:)) = surf(x5, y5, z5-groundCoordinatesZ(i,1), 'edgecolor', edgeColor, 'LineWidth', 0.1);
                        end
                        
                        % Display end effectors as spheres
%                         EEviz.(EEnames(k,:)) = surf(xNom.(EEnames(k,:))+xEE+data.(EEnames(k,:)).r.EE(i,1), ... 
%                                                     yNom.(EEnames(k,:))+yEE+data.(EEnames(k,:)).r.EE(i,2), ...
%                                                     zNom.(EEnames(k,:))+zEE+data.(EEnames(k,:)).r.EE(i,3)-groundCoordinatesZ(i,1), 'edgecolor', 'none');
                        
                        EEviz.(EEnames(k,:)) = surf(rEE(1,1) + xEE, ... 
                                                    rEE(1,2) + yEE, ...
                                                    rEE(1,3) + zEE - groundCoordinatesZ(i,1), 'edgecolor', 'none');
                                                
                                                colormap(myMap);
                    
                    end
                    hold off   
            
            %% Save gif and svg
            if mod(i,200) == 0
            end
            if createGif
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
 end