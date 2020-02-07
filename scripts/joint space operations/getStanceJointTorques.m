% Calculate stance phase torque by: torque = J'*F due to ground reaction and
% gravitational forces.
function jointTorque = getStanceJointTorques(externalForce, Leg, EEselection, meanCyclicMotionHipEE)
    
    linkCount = Leg.basicProperties.linkCount;
    gravity = -9.81; % [m/s^2]
    
    % Get J_P from HAA to EE and Jacobian from HAA to each component COM.
    if linkCount == 2
        componentName = {'EE', 'HFE', 'KFE', 'hip', 'thigh', 'shank'};
    elseif linkCount == 3
        componentName = {'EE', 'HFE', 'KFE', 'AFE', 'hip', 'thigh', 'shank', 'foot'};
    elseif linkCount == 3
        componentName = {'EE', 'HFE', 'KFE', 'AFE', 'DFE' 'hip', 'thigh', 'shank', 'foot', 'phalanges'};    
    end
    
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        selectFrontHind = 1;
    else
        selectFrontHind = 2;
    end
     
    % J'*F = torque. Compute the contribution for external EE forces due to
    % ground contact and gravitational forces on EE, link and actuator masses.
    
    for i = 1:length(Leg.(EEselection).q(:,1))
       
        torqueComponentMass(i,:) = zeros(1, linkCount+2); % Initialize as zero then add torque due to each component in the leg.
        q        = Leg.(EEselection).q(i,1:end-1);
        rotBodyY = meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2);
        
        % Get Jacobian to EE for external forces.
        [J_P, C_0EE, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(Leg, rotBodyY, q, EEselection);

        % Get Jacobians: HAA-EE, HAA-COM_link, HAA-COM_Actuator
            % In swing, return zero as the relevant torques are computed from the rigid body tree for swing phase.        
            if abs(Leg.(EEselection).force(i,3)) > 0.001 % If leg is in stance, compute torque due to the mass of leg components. Otherwise return zero for this torque.
                for k = 1:length(componentName)
                    J_P_component.([componentName{k}]) = jointToPosJacComponent(Leg, rotBodyY, q, EEselection, componentName(k));

                    % Get EE and link mass
                    if strcmp(componentName{k}, 'EE') || strcmp(componentName{k}, 'hip') || strcmp(componentName{k}, 'thigh') || strcmp(componentName{k}, 'shank') || strcmp(componentName{k}, 'foot') || strcmp(componentName{k}, 'phalanges') 
                        componentWeight = [0; 0; gravity*Leg.robotProperties.([componentName{k}])(selectFrontHind).mass]; 
                    % Get actuator mass
                    else
                        componentWeight = [0; 0; gravity*Leg.actuatorProperties.mass.([componentName{k}])]; 
                    end
                    % Compute torque due to each component weight. This is
                    % only due to the weight and not due to
                    % inertia/acceleration which is neglected.
                    torqueComponentMass(i,:) = torqueComponentMass(i,:) + (J_P_component.([componentName{k}])' * componentWeight)';
                end
            end
        
        % Compute torque due to EE forces and sum with component mass
        % torque
        torqueEEForce(i,:) = J_P' * externalForce(i,:)'; % Torque due to EE forces
        jointTorque(i,:) = torqueEEForce(i,1:end-1) + torqueComponentMass(i,1:end-1); % Last column is torque at EE which is always zero
    end
%     
%     close all
%     figure(1)
%     subplot(2,1,1)
%     plot(torqueEEForce(:,2), 'r')
%     grid on
%     hold on
%     plot(torqueComponentMass(:,2), 'b')
%     subplot(2,1,2)
%     plot(torqueEEForce(:,3), 'r')
%     hold on
%     plot(torqueComponentMass(:,3), 'b')
%     grid on
    
    % The actuators need to supply the reaction torques (ie the negative)
    jointTorque = -jointTorque;
end