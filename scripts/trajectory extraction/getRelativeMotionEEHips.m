% Input the raw motion data from the extracted towr bag. Output the
% relative motion of each end effector with respect to the leg's hip
% attachment point.
function [relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, robotProperties, trajectoryData, dt, EEnames, legCount)

    %% Calculate hip positions in inertial frame
    % First calculate hip position relative to center of mass from the nominal
    % hip position and body rotation then add the center of mass position in 
    % inertial frame. This gives the hip position in inertial frame.
    for j = 1:legCount
        EEselection = EEnames(j,:);
        for i = 1:length(quat)
            C_IBody(:,:,i) = quat2rotm(quat(i,:));  % Body rotation matrix in inertial frame.
            IF_hip.(EEselection).position(i,:) = robotProperties.nomHipPos.(EEselection)*C_IBody(:,:,i) + [trajectoryData.base.position(i,1) trajectoryData.base.position(i,2) trajectoryData.base.position(i,3)];
        end
    end

    % We now have coordinates of each foot and hip in inertial frame. The
    % difference gives the relative position and velocity of the feet 
    % relative to the hips. This simulates a fixed hip allowing observation of 
    % the foot position.
    for j = 1:legCount
        EEselection = EEnames(j,:);
        relativeMotionHipEE.(EEselection).position = trajectoryData.(EEselection).position - IF_hip.(EEselection).position;

        % Start from rest
         relativeMotionHipEE.(EEselection).velocity(1,:) = [0 0 0];
         relativeMotionHipEE.(EEselection).accel(1,:)    = [0 0 0];

         % calculate velocity and acceleration using finite difference
        for i = 2:length(trajectoryData.(EEselection).position)-1
            relativeMotionHipEE.(EEselection).velocity(i,:) = (relativeMotionHipEE.(EEselection).position(i+1,:) - relativeMotionHipEE.(EEselection).position(i,:))/dt;
        end

        % calculate acceleration by finite difference
        for i = 2:length(trajectoryData.(EEselection).position)-2
            relativeMotionHipEE.(EEselection).acceleration(i,:) = (relativeMotionHipEE.(EEselection).velocity(i+1,:) - relativeMotionHipEE.(EEselection).velocity(i,:))/dt;
        end
    end
end
