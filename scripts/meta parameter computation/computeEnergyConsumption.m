function [mechEnergy, mechEnergyPerCycle, elecEnergy, elecEnergyPerCycle] = computeEnergyConsumption(mechPower, elecPower, dt)

    % set negative power to zero, no regeneration
    mechPower(mechPower<0) = 0;
    elecPower(elecPower<0) = 0;

    %% trapezoid method for integration
    mechEnergy = dt*cumtrapz(mechPower);
    mechEnergyPerCycle = mechEnergy(end,:);
    elecEnergy = dt*cumtrapz(elecPower);
    elecEnergyPerCycle = elecEnergy(end,:);
end