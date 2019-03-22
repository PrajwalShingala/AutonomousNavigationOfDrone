function [ non_lin_constr, non_lin_eq ] = non_linear_constrain(vel, drone_pos, drone_vel, obs_pos, obs_vel, R)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% getting number of obstacles
no_of_obstacles = size(obs_pos,1);
non_lin_constr = [];

% disp(size(drone_pos))
% disp(size(drone_vel))
% disp(size(obs_pos))
% disp(size(obs_vel))

%for i = 1:no_of_obstacles
%    
%    non_lin_constr(i) = -((((drone_pos(1) - obs_pos(i,1))^2 + (drone_pos(2) - obs_pos(i,2))^2 - R^2)*((vel(1) - obs_vel(i,1))^2 ...
%                            + (vel(2) - obs_vel(i,2))^2)) - (((vel(1) - obs_vel(i,1))*(drone_pos(1) - obs_pos(i,1)) ...
%                                + (vel(2) - obs_vel(i,2))*(drone_pos(2) - obs_pos(i,2)))^2));
%end

no_of_iter = 4;
vel_inc = 0.025;
inc = -vel_inc*no_of_iter/2;


for i = 1:no_of_obstacles


   for j=1:no_of_iter
	
	obs_vel_x = obs_vel(i,1) + inc;
	obs_vel_y = obs_vel(i,2) + inc;
    
    	non_lin_constr(no_of_iter*i + j - no_of_iter) = -((((drone_pos(1) - obs_pos(i,1))^2 + (drone_pos(2) - obs_pos(i,2))^2 - R^2)*((vel(1) - obs_vel_x)^2 ...
                            + (vel(2) - obs_vel_y)^2)) - (((vel(1) - obs_vel_x)*(drone_pos(1) - obs_pos(i,1)) ...
                                + (vel(2) - obs_vel_y)*(drone_pos(2) - obs_pos(i,2)))^2));
   	inc = inc + vel_inc;
   end
end

non_lin_eq=[];

                        
end

