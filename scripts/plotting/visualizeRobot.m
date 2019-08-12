function [] = visualizeRobot(linkCount, robotProperties, Leg, meanCyclicMotionHipEE, EEselection, robotVisualization, dataExtraction, optimized, saveFiguresToPDF, fileName) 
    EEnames     = Leg.basicProperties.EEnames;
    jointNames  = Leg.basicProperties.jointNames;
    legCount    = Leg.basicProperties.legCount;
    robot       = Leg.(EEselection).rigidBodyModel;
    bodyHeight  = 0.2;
    
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        selectFrontHind = 1;
    else
        selectFrontHind = 2;
    end   
    
    if dataExtraction.averageStepsForCyclicalMotion
        finalPlottingIndex = length(Leg.(EEselection).q) - 2;
    else
        finalPlottingIndex = length(Leg.(EEselection).q);
    end

    if robotVisualization.plotAllLegs
        finalPlottingIndex = [];
        for k = 1:legCount
            finalPlottingIndex = min([finalPlottingIndex, length(Leg.(EEnames(k,:)).r.HAA)]);
        end
    end

    for i = 1:finalPlottingIndex
        if (linkCount == 2) 
            config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2), ... %body rotation about inertial y
                           Leg.(EEselection).q(i,1), ... % HAA
                           Leg.(EEselection).q(i,2), ... % HFE
                           Leg.(EEselection).q(i,3)]; % KFE

        elseif (linkCount == 3)    
            config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2), ... %body rotation about inertial y
                           Leg.(EEselection).q(i,1), ... % HAA
                           Leg.(EEselection).q(i,2), ... % HFE
                           Leg.(EEselection).q(i,3), ... % KFE
                           Leg.(EEselection).q(i,4)];    % AFE    
        elseif (linkCount == 4)
            config(i,:) = [-meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2), ... %body rotation about inertial y
                           Leg.(EEselection).q(i,1), ... % HAA
                           Leg.(EEselection).q(i,2), ... % HFE
                           Leg.(EEselection).q(i,3), ... % KFE
                           Leg.(EEselection).q(i,4), ... % AFE
                           Leg.(EEselection).q(i,5)];    % DFE
        end
    end
    
    groundCoordinatesX = [2 2 -2 -2]; % + meanCyclicMotionHipEE.(EEselection).position(:,1); % ground centered at EE position
    groundCoordinatesY = [2 -2 -2 2]; % + meanCyclicMotionHipEE.(EEselection).position(:,2);
    groundCoordinatesZ = -Leg.base.position.(EEselection)(:,3)*[1 1 1 1] - robotProperties.nomHipPos.(EEselection)(3);

    %% Display robot visualization for one leg
        % define patch shift which allows for body visualization
        if robotVisualization.plotOneLeg
            if legCount > 2
                bodyLength = 1.5*robotProperties.xNom(1);
                bodyWidth = 2*robotProperties.yNom(1);
            elseif legCount == 2
                bodyLength = 0.2;
                bodyWidth  = 2*robotProperties.yNom(1);  
            elseif legCount == 1
                bodyLength = 0.2;
                bodyWidth  = 0.2;
            end
            
            if strcmp(EEselection, 'LF')
                patchShift = [0 0 0];
            elseif strcmp(EEselection, 'LH')
                patchShift = [bodyLength 0 0];
            elseif strcmp(EEselection, 'RF')
                patchShift = [0 bodyWidth 0];
            elseif strcmp(EEselection, 'RH')
                patchShift = [bodyLength bodyWidth 0];
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
        xlim([-0.4 0.4]);
        ylim([-0.3 0.3]);
        zlim([-0.6 0.2]);  
        
%         xlim([-0.5 0.5]);
%         ylim([-0.25 0.25]);
%         zlim([-0.6 0.3]);

        if robotVisualization.plotOneLeg      
        for j = 1:robotVisualization.numberOfStepsVisualized
            for i = 1:finalPlottingIndex
                figure(f1);
                % Leg visualization
                show(robot,config(i,:), 'Frames', 'off');
                
                xlim([-0.4 0.4]);
                ylim([-0.3 0.3]);
                zlim([-0.6 0.2]); 
                
%                 xlim([-0.5 0.5]);
%                 ylim([-0.25 0.25]);
%                 zlim([-0.6 0.3]);
              
                hold on
                    if optimized && Leg.basicProperties.optimizedLegs.(EEselection)
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
                    if Leg.(EEselection).force(i,3) > 0 
                        groundColor = 'r'; % stance
                    else 
                        groundColor = 'b'; % swing
                    end
                    
                    % Plot ground - currently only works for flat terrain
                    patch(groundCoordinatesX(1,:), groundCoordinatesY(1,:), groundCoordinatesZ(i,:), groundColor, 'FaceAlpha', 0.5)

                    % Plot desired trajectory to observe tracking
                    plot3(meanCyclicMotionHipEE.(EEselection).position(1:end-2,1), ...
                          meanCyclicMotionHipEE.(EEselection).position(1:end-2,2), ...
                          meanCyclicMotionHipEE.(EEselection).position(1:end-2,3),'k', 'LineWidth', 2)                    
                     
                      % Plot end effector sphere
                        surf(xEE+Leg.(EEselection).r.EE(i,1),yEE+Leg.(EEselection).r.EE(i,2),zEE+Leg.(EEselection).r.EE(i,3), 'edgecolor','none')
                        colormap('white');

                        % Plot links as cylinders
                        if ~optimized
                            rHAA = [Leg.(EEselection).r.HAA(i,1) Leg.(EEselection).r.HAA(i,2) Leg.(EEselection).r.HAA(i,3)];
                            rHFE = [Leg.(EEselection).r.HFE(i,1) Leg.(EEselection).r.HFE(i,2) Leg.(EEselection).r.HFE(i,3)];
                            rKFE = [Leg.(EEselection).r.KFE(i,1) Leg.(EEselection).r.KFE(i,2) Leg.(EEselection).r.KFE(i,3)];
                            rAFE = [Leg.(EEselection).r.AFE(i,1) Leg.(EEselection).r.AFE(i,2) Leg.(EEselection).r.AFE(i,3)];
                            rDFE = [Leg.(EEselection).r.DFE(i,1) Leg.(EEselection).r.DFE(i,2) Leg.(EEselection).r.DFE(i,3)];
                            rEE  = [Leg.(EEselection).r.EE(i,1) Leg.(EEselection).r.EE(i,2) Leg.(EEselection).r.EE(i,3)];
                        end
                        if optimized && Leg.basicProperties.optimizedLegs.(EEselection) % If the selected leg has been optimized, use optimized link lengths
                            rHAA = [Leg.(EEselection).r.HAAOpt(i,1) Leg.(EEselection).r.HAAOpt(i,2) Leg.(EEselection).r.HAAOpt(i,3)];
                            rHFE = [Leg.(EEselection).r.HFEOpt(i,1) Leg.(EEselection).r.HFEOpt(i,2) Leg.(EEselection).r.HFEOpt(i,3)];
                            rKFE = [Leg.(EEselection).r.KFEOpt(i,1) Leg.(EEselection).r.KFEOpt(i,2) Leg.(EEselection).r.KFEOpt(i,3)];
                            rAFE = [Leg.(EEselection).r.AFEOpt(i,1) Leg.(EEselection).r.AFEOpt(i,2) Leg.(EEselection).r.AFEOpt(i,3)];
                            rDFE = [Leg.(EEselection).r.DFEOpt(i,1) Leg.(EEselection).r.DFEOpt(i,2) Leg.(EEselection).r.DFEOpt(i,3)];
                            rEE  = [Leg.(EEselection).r.EE(i,1) Leg.(EEselection).r.EE(i,2) Leg.(EEselection).r.EE(i,3)];
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
    if robotVisualization.plotAllLegs      
        for i = 1:finalPlottingIndex

            figure(f1);
            % Leg visualization
            show(robot,config(i,:), 'Frames', 'off');

            xlim([-0.5 0.5]);
            ylim([-0.3 0.3]);
            zlim([-0.6 0.4]); 

            %xlim([-0.5 0.5]);
            %ylim([-0.25 0.25]);
            %zlim([-0.6 0.3]);

            hold on

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
                    % Get coordinates for body patch
                    if legCount > 2
                    bodyLength = robotProperties.xNom(1) + robotProperties.xNom(2);
                    bodyWidth  = robotProperties.yNom(1) + robotProperties.yNom(2);
                    elseif legCount == 2
                        bodyLength = 0.2;
                        bodyWidth  = robotProperties.yNom(1) + robotProperties.yNom(1);
                    elseif legCount == 1
                        bodyLength = 0.2;
                        bodyWidth  = 0.2; 
                    end
                    patchShift = [robotProperties.xNom(1) robotProperties.yNom(1) 0];
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
                                   -sin(-config(i,1)), 0  cos(-config(i,1))];

                    % Apply body rotation to obtain new vertices
                    vert = vert * bodyRotation;
                    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
                    % Plot body patch
                    patch('Vertices',vert,'Faces',fac,'FaceColor','w', 'FaceAlpha', 0.2)    
                    
                  % Get ground color dependent on phase
                    if legCount > 1
                        if Leg.LF.force(i,3) > 0 && Leg.RF.force(i,3) > 0 % Trot, all legs stance
                            groundColor = 'm';
                        elseif (Leg.LF.force(i,3) > 0 && Leg.RF.force(i,3) == 0) % Trot, LF/LH stance
                            groundColor = 'r';
                        else % Trot, LF/LH stance
                            groundColor = 'b'; 
                        end
                    else
                        if Leg.LF.force(i,3) > 0
                            groundColor = 'r';
                        else
                            groundColor = 'b';
                        end
                    end
                    
                    patch(groundCoordinatesX(1,:), groundCoordinatesY(1,:), groundCoordinatesZ(i,:), groundColor, 'FaceAlpha', 0.5)
                    
                    for k = 1:legCount
                        EEselection = EEnames(k,:);
                        
                        % Display end effectors as spheres
                        surf(xNom.(EEselection)+xEE+Leg.(EEselection).r.EE(i,1),yNom.(EEselection)+yEE+Leg.(EEselection).r.EE(i,2),zNom.(EEselection)+zEE+Leg.(EEselection).r.EE(i,3), 'edgecolor','none')
                        colormap('white');

                          % Display links as cylinders
                        if ~optimized || (optimized && ~Leg.basicProperties.optimizedLegs.(EEselection))
                            rHAA = [xNom.(EEselection)+Leg.(EEselection).r.HAA(i,1), yNom.(EEselection)+Leg.(EEselection).r.HAA(i,2), zNom.(EEselection)+Leg.(EEselection).r.HAA(i,3)];
                            rHFE = [xNom.(EEselection)+Leg.(EEselection).r.HFE(i,1), yNom.(EEselection)+Leg.(EEselection).r.HFE(i,2), zNom.(EEselection)+Leg.(EEselection).r.HFE(i,3)];
                            rKFE = [xNom.(EEselection)+Leg.(EEselection).r.KFE(i,1), yNom.(EEselection)+Leg.(EEselection).r.KFE(i,2), zNom.(EEselection)+ Leg.(EEselection).r.KFE(i,3)];
                            rAFE = [xNom.(EEselection)+Leg.(EEselection).r.AFE(i,1), yNom.(EEselection)+Leg.(EEselection).r.AFE(i,2), zNom.(EEselection)+ Leg.(EEselection).r.AFE(i,3)];
                            rDFE = [xNom.(EEselection)+Leg.(EEselection).r.DFE(i,1), yNom.(EEselection)+Leg.(EEselection).r.DFE(i,2), zNom.(EEselection)+ Leg.(EEselection).r.DFE(i,3)];
                            rEE  = [xNom.(EEselection)+Leg.(EEselection).r.EE(i,1),  yNom.(EEselection)+Leg.(EEselection).r.EE(i,2),  zNom.(EEselection)+Leg.(EEselection).r.EE(i,3)];
                        end
                        if optimized && Leg.basicProperties.optimizedLegs.(EEselection) % If the selected leg has been optimized, use optimized link lengths
                            rHAA = [xNom.(EEselection)+Leg.(EEselection).rOpt.HAA(i,1), yNom.(EEselection)+Leg.(EEselection).rOpt.HAA(i,2), zNom.(EEselection)+ Leg.(EEselection).rOpt.HAA(i,3)];
                            rHFE = [xNom.(EEselection)+Leg.(EEselection).rOpt.HFE(i,1), yNom.(EEselection)+Leg.(EEselection).rOpt.HFE(i,2), zNom.(EEselection)+ Leg.(EEselection).rOpt.HFE(i,3)];
                            rKFE = [xNom.(EEselection)+Leg.(EEselection).rOpt.KFE(i,1), yNom.(EEselection)+Leg.(EEselection).rOpt.KFE(i,2), zNom.(EEselection)+ Leg.(EEselection).rOpt.KFE(i,3)];
                            rAFE = [xNom.(EEselection)+Leg.(EEselection).rOpt.AFE(i,1), yNom.(EEselection)+Leg.(EEselection).rOpt.AFE(i,2), zNom.(EEselection)+ Leg.(EEselection).rOpt.AFE(i,3)];
                            rDFE = [xNom.(EEselection)+Leg.(EEselection).rOpt.DFE(i,1), yNom.(EEselection)+Leg.(EEselection).rOpt.DFE(i,2), zNom.(EEselection)+ Leg.(EEselection).rOpt.DFE(i,3)];
                            rEE  = [xNom.(EEselection)+Leg.(EEselection).rOpt.EE(i,1),  yNom.(EEselection)+Leg.(EEselection).r.EE(i,2),     zNom.(EEselection)+ Leg.(EEselection).r.EE(i,3)];
                        end
                        
                        [x1,y1,z1] = cylinder2P(robotProperties.hip(selectFrontHind).radius, 10, rHAA,rHFE);
                        [x2,y2,z2] = cylinder2P(robotProperties.thigh(selectFrontHind).radius, 10,rHFE,rKFE);
                        [x3,y3,z3] = cylinder2P(robotProperties.shank(selectFrontHind).radius, 10,rKFE,rAFE);
                        surf(x1, y1, z1, 'edgecolor','none')
                        surf(x2, y2, z2, 'edgecolor','none')
                        surf(x3, y3, z3, 'edgecolor','none')   
                        
                        if linkCount > 2 
                            [x4,y4,z4] = cylinder2P(robotProperties.foot(selectFrontHind).radius, 10,rAFE,rDFE);
                            surf(x4, y4, z4, 'edgecolor','none')
                        end
                        
                        if linkCount == 4
                            [x5,y5,z5] = cylinder2P(robotProperties.phalanges(selectFrontHind).radius, 10,rDFE,rEE);
                            surf(x5, y5, z5, 'edgecolor','none')
                        end
                    end
                   hold off 
                               if mod(i,2) % Save every 2nd frame into gif
              % Capture the plot as an image 
              frame = getframe(f1); 
              im = frame2im(frame); 
              [imind,cm] = rgb2ind(im,256); 
              % Write to the GIF File 
              if i == 1 
                  imwrite(imind,cm,fileName,'gif', 'Loopcount',inf); 
              else 
                  imwrite(imind,cm, fileName,'gif','WriteMode','append'); 
              end 
            end  
                end
    end
        % Save the figure to a pdf
        warning off % warning for transparency in figure
        if saveFiguresToPDF
            export_fig results.pdf -nocrop -append
        end
end