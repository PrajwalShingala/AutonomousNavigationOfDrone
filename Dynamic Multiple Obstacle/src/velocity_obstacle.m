function [new_vel] = velocity_obstacle(drone_pos, drone_vel, obs_pos, obs_vel, R)
%Velocity_obstacle Summary of this function goes here
%   drone_pos: position of drone at current time
%   drone_vel: velocity of drone at current time
%   obs_pos: obstacle positions at current time
%   obs_vel: obstacle velocities at current time
%   R: configuration space radius
% 
% disp(drone_pos)
% disp(drone_vel)
% disp(obs_pos)
% disp(obs_vel)
% 
% disp(size(obs_pos,2));
len = size(obs_pos,2);

obs_pos_x = obs_pos(:,:,1);
obs_pos_y = obs_pos(:,:,2);

obs_vel_x = obs_vel(:,:,1);
obs_vel_y = obs_vel(:,:,2);


for i = 1: len 
    obs_pos_temp(i,:) = [obs_pos_x(1,i), obs_pos_y(1,i)]; 
    obs_vel_temp(i,:) = [obs_vel_x(1,i), obs_vel_y(1,i)];
end

obs_pos = obs_pos_temp;
obs_vel = obs_vel_temp;
%disp(R)


%% getting number of obstacles
no_of_obstacles = size(obs_pos,1);
goal_point = [20,0];
goal_dir = goal_point - drone_pos;
%% Defining cost function
cost = @(vel) (atan2(goal_dir(2),goal_dir(1)) - atan2(vel(2),vel(1)))^2; %(vel(1) - drone_vel(1))^2 + (vel(2) - drone_vel(2))^2 + 

%% Initializing velocities
vel_init = drone_vel;
new_vel = drone_vel;
%disp(vel_init);
%% Defining velocity lower and upper bounds(How much variation is allowed in [Vx, Vy] from current velocity
options = optimoptions('fmincon', 'Display','off');
lb = [-2, -2]; % [0.1 0.1];
ub = [2, 2]; % [2.0 2.0];

no_of_obstacles = size(obs_pos,1)
no_of_iter = 4;
vel_inc = 0.025;
inc = -vel_inc*no_of_iter/2;
flag = true;

for i = 1:no_of_obstacles


   for j=1:no_of_iter
	
	obs_vel_x = obs_vel(i,1) + inc;
	obs_vel_y = obs_vel(i,2) + inc;
	crit = -((((drone_pos(1) - obs_pos(i,1))^2 + (drone_pos(2) - obs_pos(i,2))^2 - R^2)*((drone_vel(1) - obs_vel_x)^2 ...
                            + (drone_vel(2) - obs_vel_y)^2)) - (((drone_vel(1) - obs_vel_x)*(drone_pos(1) - obs_pos(i,1)) ...
                                + (drone_vel(2) - obs_vel_y)*(drone_pos(2) - obs_pos(i,2)))^2));    

    	if crit > 0.1
	   %disp(crit);	
	   flag = false;
	end
   	inc = inc + vel_inc;
   end
end

if flag
   %flag
   disp('No collision detected');
   return
end

%% getting optimal velocity through optimization                        
[vel_array,cost_value] = fmincon(cost,vel_init,[],[],[],[],lb,ub, @(vel)non_linear_constrain(vel,drone_pos, drone_vel, obs_pos, obs_vel, R), options);


new_vel = vel_array;
%disp(strcat("New velocity: [", num2str(new_vel)," ]"));

end

