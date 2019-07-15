function powerQuality = computePowerQuality(power)
    % power quality = (sum P_i)^2 - sum(P_i^2)
    powerQuality = sum(sum(power)).^2 - sum(sum(power.^2));
end
