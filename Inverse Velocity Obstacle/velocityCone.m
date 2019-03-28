function [cone] = velocityCone(bot_position, bot_velocity, obs_position, obs_velocity, safe_radius)
    r = [
        bot_position(1) - obs_position(1) + 0.0001;
        bot_position(2) - obs_position(2) + 0.0001;
    ];
    v = [
        bot_velocity(1) - obs_velocity(1) + 0.0001;
        bot_velocity(2) - obs_velocity(2) + 0.0001;
    ];

    cone = ((r' * v)^2 / sum(v.^2)) - sum(r.^2) + safe_radius^2;
end
