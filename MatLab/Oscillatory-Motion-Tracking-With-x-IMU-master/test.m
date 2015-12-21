addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

% Plot
%figure('Number', 'off', 'Name', 'Gyroscope');
% figure
% hold on;
% xlabel('sample');
% ylabel('distance');
% title('High-pass filtered linear position');
% legend('X', 'Y', 'Z');
% plot(0, 'r'); %gyr first col
% plot(0, 'g');
% plot(0, 'b');
% xlabel('X');
% ylabel('Y');
% title('Position');
% legend('X', 'Y', 'Z');


%%
global accX;
global accY;
global accZ;
global gyrX;
global gyrY;
global gyrZ;
global disX;
global disY;
global disZ;
global count;
global viewX;
accX=[0];
accY=[0]; 
accZ=[0];
gyrX=[0];
gyrY=[0];
gyrZ=[0];
count=0;
viewX=0;
%axis([0 3 -300 300]);
%grid on;

%%
  
% try
%     s=serial('com6');
% catch
%     error('cant serial');
% end
% set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
% s.BytesAvailableFcnMode = 'terminator';
% s.BytesAvailableFcn = {@callback};

%fopen(s);

addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library

xIMUdata = xIMUdataClass('LoggedData/LoggedData');
gyr = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
    xIMUdata.CalInertialAndMagneticData.Gyroscope.Y...
    xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
acc = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
    xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
    xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];	% accelerometer
gyrX=gyr(:,1);
gyrY=gyr(:,2);
gyrZ=gyr(:,3);
accX=acc(:,1);
accY=acc(:,2);
accZ=acc(:,3);
calc
pause;
disp('stoped')
fclose(s);
delete(s);

% return;


