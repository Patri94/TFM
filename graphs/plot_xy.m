%Plotting xy graph (ground truth and predicted pose)

g_truth_x=csvread('g_truth_x.txt');
g_truth_y=csvread('g_truth_y.txt');
p_pose_x=csvread('p_pose_x.txt');
p_pose_y=csvread('p_pose_y.txt');

%Plotting
plot(g_truth_x(:,2),g_truth_y(:,2),color,'b');
hold on;
plot(p_pose_x(:,2),p_pose_y(:,2),color,'r');

title('Evolution of pose');
xlabel('x(meters)');
ylabel('y(meters)');
