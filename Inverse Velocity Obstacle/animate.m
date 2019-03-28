clc;
clear all;
close all;

num = 6;
agent_pos = zeros(num,2);
agent_vel = zeros(num,2);
goals = zeros(num,2);

r = 10;
x = linspace(0,2*pi,num+1);

for i=1:num
	agent_pos(i,:) = [r*sin(x(i)) r*cos(x(i))];
	goals(i,:) = [-r*sin(x(i)) -r*cos(x(i))];
end

colors = distinguishable_colors(num);

goal_err = 100;

dt = 1/30;
counter = 0;

agent_path(1,:,:) = [agent_pos];

while counter < 1000
	goal_err = 0;
	tmp_pos = zeros(num,2);
	tmp_vel = zeros(num,2);
	parfor i = 1:num
		obs_pos = [];
		obs_vel = [];
		for j = 1:num
			if i == 1 && sensorRange(agent_pos(i,:), agent_vel(i,:), agent_pos(j,:))
				obs_pos = [obs_pos; agent_pos(j,:) - agent_pos(i,:)];
				obs_vel = [obs_vel; agent_vel(j,:) - agent_vel(i,:)];
			end
		end
		tmp = [0, 0];
		controls = getControls(agent_pos(i,:), agent_vel(i,:), obs_pos, obs_vel, goals(i,:), 2.5);
		if sum((goals(i,:)-agent_pos(i,:)).^2) < 1
			w = (sum((goals(i,:)-agent_pos(i,:)).^2))*0.5;
		else
			w = 1;
		end
		tmp_vel(i,:) = w*capVelocity(agent_vel(i,:) + 0.5*controls');
		tmp_pos(i,:) = agent_pos(i,:) + tmp_vel(i,:)*dt;
		% scatter(agent_pos(i,1), agent_pos(i,2));
		% filledCircle(tmp_pos(i,:), 0.9, 1000, colors(i,:));
	end
	counter = counter + 1;
	agent_pos = tmp_pos;
	agent_vel = tmp_vel
	agent_path(counter, :, :) = agent_pos;
	for i = 1:num
		filledCircle(agent_pos(i,:), 0.9, 1000, colors(i,:));
		hold on;
		p = plot(agent_path(:,i,1), agent_path(:,i,2));
		p.Color = colors(i,:);
	end
	grid on;
	axis([-11 11 -11 11]);
	drawnow;
	saveas(gcf, ['run/', num2str(counter,'%04.f'), '.png']);
	clf
end
