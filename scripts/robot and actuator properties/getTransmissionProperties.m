function  transmissionMass = getTransmissionProperties(transmissionMethod, actuateJointDirectly, robot, robotSelection, jointNames, linkNames, linkCount)
% Based on the selected transmission method and whether the joint is
% actuated remotely or directly, compute the additional mass on all preceding links.

% For example, if there is a chain transmission to remotely actuate the AFE 
% joint, the mass of this chain is applied to the shank, thigh and hip.

    % Set the mass density depending on the belt/cable/chain that is used.
    transmissionDensity.chain = 0.03; % kg/m
    transmissionDensity.belt  = 0.015; % kg/m ex Maedler belt T2.5
    transmissionDensity.cable = 0.01; % kg/ms

    % There is never a transmission mass for the hip because there is no
    % link distance over which the transmission is applied.
    transmissionMass.hip(1) = 0; transmissionMass.hip(2) = 0;     

    % Initialize transmission mass as zero and add the mass of the
    % transmission as applicable.
    transmissionMass.thigh(1)     = 0;  transmissionMass.thigh(2)     = 0;     
    transmissionMass.shank(1)     = 0;  transmissionMass.shank(2)     = 0;     
    transmissionMass.foot(1)      = 0;  transmissionMass.foot(2)      = 0;     
    transmissionMass.phalanges(1) = 0;  transmissionMass.phalanges(2) = 0;     
    
    if robot.(robotSelection).legCount > 2
        frontHindIndex = 2; % Robot has front and hind legs. ie >2 legs
    else
        frontHindIndex = 1; % Robot only has 'front' legs. ie has <3 legs
    end
    
    % The transmission mass is applied to all links preceding the remotely
    % actuated joint.
    for i = 2:linkCount+1 % neglect HAA because there is no link preceding the HAA joint. Start instead at HFE.
        for j = 1:frontHindIndex % j indexes the front and hind leg  
            if actuateJointDirectly.(jointNames(i,:))
                transmissionMass.(linkNames{i})(j) = transmissionMass.(linkNames{i})(j); % transmission mass unchanged if directly actuated
            else 
                % Additional mass of transmission length * density is
                % applied to ALL preceding links.
                density = transmissionDensity.(transmissionMethod.(jointNames(i,:))); % Density of the applied transmission method
                k = i;
                while k > 1 
                    transmissionMass.(linkNames{k-1})(j) = robot.(robotSelection).(linkNames{k-1})(j).length * density; 
                    k = k - 1;
                end
            end
        end
    end
end