function [c, ceq] = getConstraints(bot_position, u, bot_controls, obstacle_position, obstacle_velocity, safe_radius)
    c = zeros(size(obstacle_position,1)+2,1);
    for i = 1:size(obstacle_position,1)
	tmp = (obstacle_position(i,:,:));
        c(i) = velocityCone(bot_position, u, tmp, obstacle_velocity(i,:,:), safe_radius);
    end
    c(i+1) = (u(1)+bot_controls(1))^2 - 2;
    c(i+2) = (u(2)+bot_controls(2))^2 - 2;
    ceq = [];
end
