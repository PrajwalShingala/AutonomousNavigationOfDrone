function [future_position] = futureStates(bot_position, bot_velocity)
    dt = 1/20;
    future_position = bot_position + bot_velocity*dt;
end
