%Plotting data from error topic

%Loading files
XYError=csvread('error_D.txt');
yaw_error=csvread('error_Y.txt');

%%Starting from time 0
XYError(:,1)=XYError(:,1)-XYError(1,1);
yaw_error(:,1)=yaw_error(:,1)-yaw_error(1,1);

%from nsecs to secs
XYError(:,1)=XYError(:,1)/1000000000;
yaw_error(:,1)=yaw_error(:,1)/1000000000;


%Plotting 
subplot(1,2,1);
plot(XYError(:,1),XYError(:,2));
title('Absolute Error');
xlabel('time(seconds)');
ylabel('Error(meters)');

subplot(1,2,2);
plot(yaw_error(:,1),yaw_error(:,2));
title('Yaw Error');
xlabel('time(seconds)');
ylabel('Error(radians)');

