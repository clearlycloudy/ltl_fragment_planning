%% sample_task_set_1
%run contains 3 cycles of task set completions

run = table2array(sampletaskset3(:,1:5));
t = run(:,1);
agent_pos = run(:,2:3);
moving_obs_pos = run(:,4:5);

rectangle('Position',[1 0 8 9], 'EdgeColor', 'y', 'FaceColor', 'y')
text(1,9,'persistence area','Color','y');
hold on;

plot(agent_pos(:,1), agent_pos(:,2),'k')
hold on;
plot(moving_obs_pos(:,1), moving_obs_pos(:,2),'r')
xlim([-1,10])
ylim([-1,10])
grid on;
hold on;
text(0,0,'agent start','Color','k')
plot(0,0,'o');
hold on;
text(7,7,'moving obstruction start','Color','r')
plot(7,7,'*');

plot(9,0,'sm');
text(9,0,'task', 'Color', 'm')
plot(9,9,'sm');
text(9,9,'task', 'Color', 'm')
plot(5,9,'sm');
text(5,9,'task', 'Color', 'm')

plot(3,0,'xb');
plot(3,1,'xb');
text(3,0,'next step resp', 'Color', 'b')
text(3,1,'next step resp', 'Color', 'b')

plot(9,5,'xr');
text(9,5,'stationary obstacle', 'Color', 'r')

