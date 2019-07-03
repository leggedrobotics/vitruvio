function powerQuality = computePowerQuality(power, EEselection)

% power quality = (sum P_i)^2 - sum(P_i^2)

powerQuality.(EEselection) = sum(sum(power)).^2 - sum(sum(power.^2));
end
