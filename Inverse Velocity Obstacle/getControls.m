function [controls] = getControls(bot_position, bot_controls, obstacle_position, obstacle_velocity, goal, safe_radius)
    cost = @(u) sum((goal - futureStates(bot_position, u'+bot_controls)) .^2) + 0.5*sum(u.^2);
    
    init = [0;0];
    lb = [-0.5; -0.5];
    ub = [0.5; 0.5];
    if size(obstacle_position, 1) == 0
        constraints = @(u) velConstraints(init, u, bot_controls, obstacle_position, obstacle_velocity, safe_radius);
    else
        constraints = @(u) getConstraints(init, u, bot_controls, obstacle_position, obstacle_velocity, safe_radius);
    end
    options = optimoptions('fmincon','Display','off','Algorithm','active-set', 'UseParallel', true);
    controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options);
end
