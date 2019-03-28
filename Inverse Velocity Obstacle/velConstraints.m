function [c, ceq] = getConstraints(bot_position, u, bot_controls, obstacle_position, obstacle_velocity, safe_radius)
    c = [(u(1)+bot_controls(1))^2 - 2;
	(u(2)+bot_controls(2))^2 - 2;];
    ceq = [];
end
