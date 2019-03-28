clc;
clear all;
close all;

num = 2;
samples = 2000;
agent_pos = zeros(num*samples,2);
agent_vel = zeros(num*samples,2);
goals = zeros(num*samples,2);
pos_noise = sqrt(0.005)*randn(num*samples,2);

r = 10;
x = linspace(0,2*pi,num+1);

for i=1:num
	for j=j:samples
		agent_pos(i*samples+j,:) = [r*sin(x(i)) r*cos(x(i))] + [pos_noise(i*samples+j,1) pos_noise(i*samples+j,2)];
		goals(i,:) = [-r*sin(x(i)) -r*cos(x(i))];
	end
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
	for i = 1:samples
		obs_pos = [];
		obs_vel = [];
		if sensorRange(agent_pos(i,:), agent_vel(i,:), agent_pos(samples+i,:))
			obs_pos = [obs_pos; agent_pos(samples+i,:) - mean(agent_pos(1:samples,:))];
			obs_vel = [obs_vel; agent_vel(samples+i,:) - mean(agent_vel(1:samples,:))];
		end
		tmp = [0, 0];
		controls = getControls(agent_pos(i,:), mean(agent_vel(i,:)), obs_pos, obs_vel, goals(i,:), 2.5);
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
		filledCircle(mean(agent_pos(1:samples,:)), 0.9, 1000, colors(i,:));
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
