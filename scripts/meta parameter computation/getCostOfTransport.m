function CoT = getCostOfTransport(Leg, power, quadruped, EEselection)
% CoT = power / m*g*v

m = quadruped.mass.total;
g = 9.81;
% mean velocity in x direction
v = mean(Leg.CoM.velocity(:,1));

% set negative power to zero, no regeneration
for j = 1:length(power)
    for k = 1:length(power(1,:))
        if power(j,k) < 0
              power(j,k) = 0;
        end
    end
end

totalJointPower = sum(mean(power));
CoT = totalJointPower/(m*g*v);
