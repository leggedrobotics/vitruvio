function [actuatorEfficiency, elecPower] = getActuatorEfficiency(Leg, EEnames, linkCount, actuatorProperties);
% optimal efficiency at qdotOpt, torqueOpt.
% torqueMax = actuatorProperties.maxTorqueLimit;
% qdotMax = actuatorProperties.maxqdotLimit;
% mechPowerMax = actuatorProperties.maxPowerLimit;
% qdotOpt = 0.7*qdotMax;
% torqueOpt = 0.7*torqueMax;
% optimalOperatingPoint = [qdotOpt, torqueOpt];

torqueMax = 150; % [Nm]
qdotMax = 12; % [rad/s]
mechPowerMax = 60; % [kW]
qdotOpt   = 0.5*qdotMax;
torqueOpt = 0.5*torqueMax;
optimalOperatingPoint = [qdotOpt, torqueOpt];

%% compute torque envelope 
torqueEnvelope = zeros(length(0:0.01:qdotMax),1);
qdotIntercept = mechPowerMax/torqueMax * 60/(2*pi); % [kr/min]
i = 1;
% constant torque region
for qdotEnvelope = 0:0.01:qdotIntercept
    torqueEnvelope(i,1) = torqueMax;
    i = i+1;
end
% constant power region
for qdotEnvelope = qdotIntercept+0.01:0.01:qdotMax
    torqueEnvelope(i,1) = mechPowerMax/(qdotEnvelope*2*pi/60); % [Nm]
    i = i+1;
end

qdotEnvelope = [0:0.01:qdotMax]';

figure('name', 'Torque Envelope', 'DefaultAxesFontSize', 10)
hold on
plot(qdotEnvelope, torqueEnvelope, 'k', 'lineWidth', 2)
plot(optimalOperatingPoint(1,1), optimalOperatingPoint(1,2), 'x')
xlabel('Torque [Nm]')
ylabel('qdot [rad/s]')
hold off

%% approximate iso efficiency using technique from Efficiency Maps of Electrical Machines
meshDimension = 50;
qdot = linspace(0, qdotMax, meshDimension); % [kr/min]
torque = (linspace(0, torqueMax, meshDimension))'; % [Nm]

% normalized power loss terms
P_loss(:,:,1)  = ones(meshDimension,meshDimension);
P_loss(:,:,2)  = (qdot/qdotMax).*(ones(meshDimension,1));
P_loss(:,:,3)  = ((qdot/qdotMax).^2).*(ones(meshDimension,1));
P_loss(:,:,4)  = ((qdot/qdotMax).^3).*(ones(meshDimension,1));

P_loss(:,:,5)  = (torque/torqueMax).*(ones(1,meshDimension));
P_loss(:,:,6)  = (torque/torqueMax).*(qdot/qdotMax); 
P_loss(:,:,7)  = (torque/torqueMax).*(qdot/qdotMax).^2;
P_loss(:,:,8)  = (torque/torqueMax).*(qdot/qdotMax).^3;

P_loss(:,:,9)   = ((torque/torqueMax).^2).*(ones(1,meshDimension));
P_loss(:,:,10)  = ((torque/torqueMax).^2).*(qdot/qdotMax); 
P_loss(:,:,11)  = ((torque/torqueMax).^2).*(qdot/qdotMax).^2;
P_loss(:,:,12)  = ((torque/torqueMax).^2).*(qdot/qdotMax).^3;

P_loss(:,:,13)   = ((torque/torqueMax).^3).*(ones(1,meshDimension));
P_loss(:,:,14)  = ((torque/torqueMax).^3).*(qdot/qdotMax); 
P_loss(:,:,15)  = ((torque/torqueMax).^3).*(qdot/qdotMax).^2;
P_loss(:,:,16)  = ((torque/torqueMax).^3).*(qdot/qdotMax).^3;

% to align subplots with their positions in reference paper for comparison
subplotIndex           = [13 14 15 16 9  10 11 12 5 6  7  8  1 2 3 4];

figure('name', 'Power Losses', 'DefaultAxesFontSize', 10)
for i = 1:16
    subplot(4, 4, subplotIndex(i))
    surf(qdot/qdotMax, torque/torqueMax, P_loss(:,:,i))
    xlabel('qdot');
    ylabel('torque')
    shading interp
    colormap hot
    view(2)
end

figure('name', 'Efficiency', 'DefaultAxesFontSize', 10)
for i = 1:16
    efficiency(:,:,i) = ((qdot/qdotMax).*(torque/torqueMax))./(((qdot/qdotMax).*(torque/torqueMax)) + P_loss(:,:,i));
    subplot(4, 4, subplotIndex(i))
    surf(qdot/qdotMax, torque/torqueMax, efficiency(:,:,i))
    xlabel('qdot');
    ylabel('torque')
    shading interp
    colormap hot
    view(2)    
end
         
%% the total P_loss is a weighted sum of the different loss terms
% k_mn*T^m*qdot^n
qdotBase = 12; %[kr/min]
torqueBase = 250; % [Nm]
PowerBase = 8; % [kW]

%% Normalized to base qdot, torque
P_loss(:,:,1)  = ones(meshDimension,meshDimension);
P_loss(:,:,2)  = (qdot/qdotBase).*(ones(meshDimension,1));
P_loss(:,:,3)  = ((qdot/qdotBase).^2).*(ones(meshDimension,1));
P_loss(:,:,4)  = ((qdot/qdotBase).^3).*(ones(meshDimension,1));

P_loss(:,:,5)  = (torque/torqueBase).*(ones(1,meshDimension));
P_loss(:,:,6)  = (torque/torqueBase).*(qdot/qdotBase); 
P_loss(:,:,7)  = (torque/torqueBase).*(qdot/qdotBase).^2;
P_loss(:,:,8)  = (torque/torqueBase).*(qdot/qdotBase).^3;

P_loss(:,:,9)   = ((torque/torqueBase).^2).*(ones(1,meshDimension));
P_loss(:,:,10)  = ((torque/torqueBase).^2).*(qdot/qdotBase); 
P_loss(:,:,11)  = ((torque/torqueBase).^2).*(qdot/qdotBase).^2;
P_loss(:,:,12)  = ((torque/torqueBase).^2).*(qdot/qdotBase).^3;

P_loss(:,:,13)   = ((torque/torqueBase).^3).*(ones(1,meshDimension));
P_loss(:,:,14)  = ((torque/torqueBase).^3).*(qdot/qdotBase); 
P_loss(:,:,15)  = ((torque/torqueBase).^3).*(qdot/qdotBase).^2;
P_loss(:,:,16)  = ((torque/torqueBase).^3).*(qdot/qdotBase).^3;

motorType = 'SPM';

if strcmp(motorType,'SPM')
    k_00 = -0.002;
    k_01 = 0.175;
    k_02 = 0.181;
    k_03 = 0.443;

    k_10 = -0.065;
    k_11 = 0.577;
    k_12 = -0.542;
    k_13 = 0;

    k_20 = 0.697;
    k_21 = -1.043;
    k_22 = 0;
    k_23 = 0;

    k_30 = 0.942;
    k_31 = 0;
    k_32 = 0;
    k_33 = 0;
    
else
    k_00 = 1;
    k_01 = 1;
    k_02 = 1;
    k_03 = 1;

    k_10 = 1;
    k_11 = 1;
    k_12 = 1;
    k_13 = 1;

    k_20 = 1;
    k_21 = 1;
    k_22 = 1;
    k_23 = 1;

    k_30 = 1;
    k_31 = 1;
    k_32 = 1;
    k_33 = 1;
end
   
P_lossTotal = k_00 * P_loss(:,:,1)  + ...
              k_01 * P_loss(:,:,2)  + ...
              k_02 * P_loss(:,:,3)  + ...
              k_03 * P_loss(:,:,4)  + ...
              k_10 * P_loss(:,:,5)  + ...
              k_11 * P_loss(:,:,6)  + ...
              k_12 * P_loss(:,:,7)  + ...
              k_13 * P_loss(:,:,8)  + ...
              k_20 * P_loss(:,:,9)  + ...
              k_21 * P_loss(:,:,10) + ...
              k_22 * P_loss(:,:,11) + ...
              k_23 * P_loss(:,:,12) + ...
              k_30 * P_loss(:,:,13) + ...
              k_31 * P_loss(:,:,14) + ...
              k_32 * P_loss(:,:,15) + ...
              k_33 * P_loss(:,:,16);

figure('name', 'Total power losses', 'DefaultAxesFontSize', 10)
hold on
contour(qdot, torque, P_lossTotal)
plot(qdotEnvelope, torqueEnvelope, 'k', 'lineWidth', 2)
plot(optimalOperatingPoint(1,1), optimalOperatingPoint(1,2), 'o')
hold off

% efficiencyTotal = ((qdot*2*pi/60.*torque)/PowerBase)./(((qdot*2*pi/60.*torque)/PowerBase) + P_lossTotal);
efficiencyTotal = 1./(1 + P_lossTotal);

figure('name', 'Overall efficiency', 'DefaultAxesFontSize', 10)
hold on
contour(qdot, torque, efficiencyTotal)
plot(qdotEnvelope, torqueEnvelope, 'k', 'lineWidth', 2)
plot(optimalOperatingPoint(1,1), optimalOperatingPoint(1,2), 'o')
hold off

end
