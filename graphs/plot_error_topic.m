%Plotting data from error topic

%Loading files
XYError=csvread('error_D.txt');
yaw_error=csvread('error_Yaw.txt');
markers=csvread('markers.txt');
ErrorX=csvread('error_X.txt');
ErrorY=csvread('error_Y.txt');
valorD=max(XYError(2:end,2));
valorY=max(yaw_error(2:end,2));
valorCY=max(ErrorY(2:end,2));
%Starting from time 0
XYError(:,1)=XYError(:,1)-XYError(1,1);
yaw_error(:,1)=yaw_error(:,1)-yaw_error(1,1);
num=markers(:,2);
num(num >= 1)=valorD;
markers(:,1)= markers(:,1)-markers(1,1);
ErrorX(:,1)=ErrorX(:,1)-ErrorX(1,1);
ErrorY(:,1)=ErrorY(:,1)-ErrorY(1,1);

%from nsecs to secs
XYError(:,1)=XYError(:,1)/1000000000;
yaw_error(:,1)=yaw_error(:,1)/1000000000;
markers(:,1)=markers(:,1)/1000000000;
ErrorX(:,1)=ErrorX(:,1)/1000000000;
ErrorY(:,1)=ErrorY(:,1)/1000000000;

%from radians to degrees
yaw_error(:,2)=yaw_error(:,2)*(360/(2*pi));

%Plotting
subplot(2,2,1);
plot(XYError(2:end,1),XYError(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),num(2:end),'color','r');
%plot(markers(2:end,1),markers(2:end,2));
title('Absolute Error');
xlabel('time(seconds)');
ylabel('Error(meters)');
%legend('Error','Number of markers');

subplot(2,2,2);
plot(ErrorX(2:end,1),ErrorX(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),num(2:end),'color','r');
%plot(markers(2:end,1),markers(2:end,2));
title('Error X');
xlabel('time(seconds)');
ylabel('Error(meters)');
%legend('Error','Number of markers');
numy=markers(:,2);
numy(numy >= 1)=valorCY;
subplot(2,2,3);
plot(ErrorY(2:end,1),ErrorY(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),num(2:end),'color','r');
%plot(markers(2:end,1),markers(2:end,2));
title('Error Y');
xlabel('time(seconds)');
ylabel('Error(meters)');
%legend('Error','Number of markers');

numyaw=markers(:,2);
numyaw(numyaw >= 1)=valorY;

subplot(2,2,4);
plot(yaw_error(2:end,1),yaw_error(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),num(2:end),'color','r');
%plot(markers(2:end,1),markers(2:end,2));
title('Yaw Error');
xlabel('time(seconds)');
ylabel('Error(degrees)');
%legend('Error','Number of markers');


figure;
plot(markers(2:end,1),markers(2:end,2));
title('Number of visual markers');
xlabel('time(seconds)');
ylabel('Number');


