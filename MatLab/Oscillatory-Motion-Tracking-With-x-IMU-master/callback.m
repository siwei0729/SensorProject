 %%
function callback(s, BytesAvailable)
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
out = fscanf(s);
%disp(out)
test = strsplit(out,'IFO');
 
if(strcmp(test{1},'F')==1)
    return
end
C = strsplit(out,',');

ax = str2num(C{1});
ay = str2num(C{2});
az = str2num(C{3});
% gx = str2num(C{4});
% gy = str2num(C{5});
% gz = str2num(C{6});

if(length(accX)>600)
    cla
    accX = [accX(length(accX)-5:length(accX)) ax];
    accY = [accY(length(accY)-5:length(accY)) ay];
    accZ = [accZ(length(accZ)-5:length(accZ)) az];

%     gyrX = [gyrX(length(gyrX)-5:length(gyrX)) gx];
%     gyrY = [gyrY(length(gyrY)-5:length(gyrY)) gy];
%     gyrZ = [gyrZ(length(gyrZ)-5:length(gyrZ)) gz];
    disX=[0];
    disY=[0];
    disZ=[0];
    
else
    accX = [accX ax];
    accY = [accY ay];
    accZ = [accZ az];
%     gyrX = [gyrX gx];
%     gyrY = [gyrY gy];
%     gyrZ = [gyrZ gz];
end

count=count+1;
if(count>5)
    scatter3(accX,accY,accZ)
    count=0;    
end
% if(length(accX)>10) 
%     calc
% end
% count = count+1;
% plot(accX,'r');
% plot(accY,'g');
% plot(accZ,'b');
% plot(gyrX,'y');
% plot(gyrY,'m');
% plot(gyrZ,'c');
%plot(ax,ay,az)
%set(p, 'XData',allX,'YData',allY);

%drawnow

%axis([x-200 x+200 -90 90]);
end