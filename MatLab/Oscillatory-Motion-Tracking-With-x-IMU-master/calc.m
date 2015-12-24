 function calc()
%% Import data
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
% close all;                     	% close all figures
% clear;                         	% clear all variables
% clc;                          	% clear the command terminal

% xIMUdata = xIMUdataClass('LoggedData/LoggedData');
% gyr = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
%     xIMUdata.CalInertialAndMagneticData.Gyroscope.Y... 
%     xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
% acc = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
%     xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
%     xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];	% accelerometer

global accX;
global accY;
global accZ;
global gyrX;
global gyrY;
global gyrZ;
global disX;
global disY;
global disZ;

gyr = [gyrX(:) gyrY(:) gyrZ(:)];        % gyroscope
acc = [accX(:) accY(:) accZ(:)];	% accelerometer
  gyr = arrSplit(gyr,2000,10000);
  acc = arrSplit(acc,2000,10000);
% gyr = [gyr(1600:11000,1) gyr(1600:11000,2) gyr(1600:11000,3)];
% acc = [acc(1600:11000,1) acc(1600:11000,2) acc(1600:11000,3)];
samplePeriod = 1/256; % 1/256
FPS = 256;


% Plot
%figure('Number', 'off', 'Name', 'Gyroscope');
% figure();
% hold on;
% plot(gyr(:,1), 'r'); %gyr first col
% plot(gyr(:,2), 'g');
% plot(gyr(:,3), 'b');
% xlabel('sample');
% ylabel('dps');
% title('Gyroscope');
% legend('X', 'Y', 'Z');

%figure('Number', 'off', 'Name', 'Accelerometer');
figure()
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));	% gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc));  % accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)'; % shift all gravity projection into Z axis, HighPass filter!!!
end

% Plot
%figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
figure()
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s



% Plot
%figure('Number', 'off', 'Name', 'Linear Acceleration');
figure()
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% High-pass filter linear acceleration
% order = 1;
% filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% linAccHP = filtfilt(b, a, linAcc);
% figure()
%
% hold on;
% plot(linAccHP(:,1), 'r');
% plot(linAccHP(:,2), 'g');
% plot(linAccHP(:,3), 'b');
% xlabel('sample');
% ylabel('g');
% title('Linear Acc High Passed');
% legend('X', 'Y', 'Z');
%
% linAcc = linAccHP;
%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));
% count =0;
for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end
    %% Above commented work totally failed, creating a lot drifts
% Plot
%figure('Number', 'off', 'Name', 'Linear Velocity');
figure()
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');
grid on
spectrum(linVel(:,1),FPS,'X Spectrum')
spectrum(linVel(:,2),FPS,'Y Spectrum')
spectrum(linVel(:,3),FPS,'Z Spectrum')
%pause()

%% High-pass filter linear velocity to remove drift
fre = getCutOffFre(linVel(:,1),FPS);
disp('X cut off fre:')
disp(fre)
order = 1;
filtCutOff = fre; %cut off f at x hz
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high'); % 1/5 of its f
linVelHP(:,1) = filtfilt(b, a, linVel(:,1));

fre = getCutOffFre(linVel(:,2),FPS);
disp('Y cut off fre:')
disp(fre)
order = 1;
filtCutOff = fre; %cut off f at x hz
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high'); % 1/5 of its f
linVelHP(:,2) = filtfilt(b, a, linVel(:,2));

fre = getCutOffFre(linVel(:,3),FPS);
disp('Z cut off fre:')
disp(fre)
order = 1;
filtCutOff = fre; %cut off f at x hz
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high'); % 1/5 of its f
linVelHP(:,3) = filtfilt(b, a, linVel(:,3));

%% linVelHP = filtfilt(b, a, linVel); original
%linVelHP = linVel;
% Plot
%figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');
figure()
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');
grid on

for i = 1:20:length(linAcc)
%     disp('--------------------- ')
%     disp(i)
    countX =0;
%------------X-----------
    for k = i:1:i+19
        if k<length(linAcc) && abs(linAcc(k,1))<0.4 && abs(linAcc(k,2))<0.4 && abs(linAcc(k,3))<0.4
            countX = countX +1;
        end
    end
    if(countX>=15)
        for k = i:2:i+19
            linVelHP(k,1)=-0.001;
            linVelHP(k+1,1)=0.001;
            linVelHP(k,2)=-0.001;
            linVelHP(k+1,2)=0.001;
            linVelHP(k,3)=-0.001;
            linVelHP(k+1,3)=0.001;
        end
    end
    %disp('countX: '+countX)

end

figure()
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');
grid on

spectrum(linVelHP(:,1),FPS,'X Spectrum')
spectrum(linVelHP(:,2),FPS,'Y Spectrum')
spectrum(linVelHP(:,3),FPS,'Z Spectrum')

%% Calculate linear position (integrate velocity)

linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
%figure('Number', 'off', 'Name', 'Linear Position');
figure()
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');
spectrum(linPos(:,1),FPS,'X Pos Spectrum')
spectrum(linPos(:,2),FPS,'Y Pos Spectrum')
spectrum(linPos(:,3),FPS,'Z Pos Spectrum')
linPosHP = linPos;
%% High-pass filter linear position to remove drift

% order = 1;
% filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% linPosHP = filtfilt(b, a, linPos);
% spectrum(linPosHP(:,1),FPS,'X PosHP Spectrum')
% spectrum(linPosHP(:,2),FPS,'Y PosHP Spectrum')
% spectrum(linPosHP(:,3),FPS,'Z PosHP Spectrum')
%linPosHP = linPos;
% linPosHP(:,1) = fliplr(linPosHP(:,1));
% linPosHP(:,2) = fliplr(linPosHP(:,2));
% linPosHP(:,3) = fliplr(linPosHP(:,3));
% Plot
%figure('Number', 'off', 'Name', 'High-pass filtered Linear Position');
% figure()
% hold on;
% plot(linPosHP(:,1), 'r');
% plot(linPosHP(:,2), 'g');
% plot(linPosHP(:,3), 'b');
% xlabel('sample');
% ylabel('g');
% title('High-pass filtered linear position');
% legend('X', 'Y', 'Z');

% cla
% sumAll = sum(linPosHP(:,1));
% disX = [disX sumAll];
% sumAll = sum(linPosHP(:,2));
% disY = [disY sumAll];
% sumAll = sum(linPosHP(:,3));
% disZ = [disZ sumAll];
% figure()

% scatter3(linPosHP(:,1),linPosHP(:,2),linPosHP(:,3))
% title('High-pass filtered linear position in 3D');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

%view(20,40)
% angle = 0;
% while(1)
%     pause(0.03);
%     angle = angle +0.3;
%     if(angle>=360)
%         angle=0;
%     end
%     view(angle,40)
% end
% veloVeri= zeros(size(linPosHP));
% for i=2 : length(linPosHP)
%     veloVeri(i,1) = (linPosHP(i,1)-linPosHP(i-1,1))*FPS; %(d2-d1)/t
%     veloVeri(i,2) = (linPosHP(i,2)-linPosHP(i-1,2))*FPS;
%     veloVeri(i,3) = (linPosHP(i,3)-linPosHP(i-1,3))*FPS;
% end
% figure()
% hold on;
% plot(veloVeri(:,1), 'r');
% plot(veloVeri(:,2), 'g');
% plot(veloVeri(:,3), 'b');
% xlabel('sample');
% ylabel('velocity');
% title('velocity verify');
% legend('X', 'Y', 'Z');
% accVeri= zeros(size(linPosHP));
% for i=3 : length(veloVeri)
%     accVeri(i,1) = (veloVeri(i,1)-veloVeri(i-1,1))*FPS; %(v2-v1)/t
%     accVeri(i,2) = (veloVeri(i,2)-veloVeri(i-1,2))*FPS;
%     accVeri(i,3) = (veloVeri(i,3)-veloVeri(i-1,3))*FPS;
% end
% figure()
% hold on;
% plot(accVeri(:,1), 'r');
% plot(accVeri(:,2), 'g');
% plot(accVeri(:,3), 'b');
% xlabel('sample');
% ylabel('acceleration');
% title('acceleration verify');
% legend('X', 'Y', 'Z');
% disp('----------------------')
% disp(linPosHP(1,1))
% disp(linPosHP(1,2))
% disp(linPosHP(1,3))
%view(-30,20)
%axis([-1 1 -1 1 -1 1]);

%linPosHP = [linPosHP(1:11000,1) linPosHP(1:11000,2) linPosHP(1:11000,3)];
%% Play animation

SamplePlotFreq = 24; %8

SixDOFanimation(linPosHP, R, ...
    'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
    'Position', [9 39 1280 720], ...
    'AxisLength', 0.1, 'ShowArrowHead', false, ...
    'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
    'CreateAVI', true, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));

%% End of script
end