%%
%配套boyi3
%画图代码
x=red_position(1,:);
y=red_position(2,:);
z=red_position(3,:);

arrowX = diff(x);
arrowY = diff(y);
arrowZ = diff(z);
arrowMag = sqrt(arrowX.^2 + arrowY.^2 + arrowZ.^2);  % 矢量大小
arrowX = arrowX ./ arrowMag;  % 归一化
arrowY = arrowY ./ arrowMag;
arrowZ = arrowZ ./ arrowMag;

% 创建箭头的起点
arrowStartX = x(1:end-1);
arrowStartY = y(1:end-1);
arrowStartZ = z(1:end-1);
scaleFactor = 0.5;  % 缩放因子
arrowX = arrowX * scaleFactor;
arrowY = arrowY * scaleFactor;
arrowZ = arrowZ * scaleFactor;
quiver3(arrowStartX, arrowStartY, arrowStartZ, arrowX, arrowY, arrowZ, 'r','AutoScale', 'on', 'AutoScaleFactor', 0.5);
hold on
x=blue_position(1,:);
y=blue_position(2,:);
z=blue_position(3,:);
arrowX = diff(x);
arrowY = diff(y);
arrowZ = diff(z);
arrowMag = sqrt(arrowX.^2 + arrowY.^2 + arrowZ.^2);  % 矢量大小
arrowX = arrowX ./ arrowMag;  % 归一化
arrowY = arrowY ./ arrowMag;
arrowZ = arrowZ ./ arrowMag;

% 创建箭头的起点
arrowStartX = x(1:end-1);
arrowStartY = y(1:end-1);
arrowStartZ = z(1:end-1);
scaleFactor = 0.5;  % 缩放因子
arrowX = arrowX * scaleFactor;
arrowY = arrowY * scaleFactor;
arrowZ = arrowZ * scaleFactor;
quiver3(arrowStartX, arrowStartY, arrowStartZ, arrowX, arrowY, arrowZ, 'b','AutoScale', 'on', 'AutoScaleFactor', 0.5);
hold on
plot3(red_position(1,:),red_position(2,:),red_position(3,:),'r')
hold on
plot3(blue_position(1,:),blue_position(2,:),blue_position(3,:),'bl')

red = [42100 0 0.1 92 0 280];
blue = [42166 0 0.1 92 0 280];
[Red_r,Red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
[Blue_r,Blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
Red_rv=[Red_r;Red_v];
Blue_rv=[Blue_r;Blue_v];
red_position_2=[Red_rv];
blue_position_2 = [Blue_rv];

for i=1:30
Red_rv = twoBodyOrbitRV(Red_rv,predict_time);
Blue_rv = twoBodyOrbitRV(Blue_rv,predict_time);
red_position_2=[red_position_2,Red_rv];
blue_position_2=[blue_position_2,Blue_rv];
end

plot3(red_position(1,:),red_position(2,:),red_position(3,:),'r')
hold on
plot3(blue_position(1,:),blue_position(2,:),blue_position(3,:),'bl')
hold on
plot3(red_position_2(1,:),red_position_2(2,:),red_position_2(3,:),'g')
hold on
plot3(blue_position_2(1,:),blue_position_2(2,:),blue_position_2(3,:),'k')
distance = []
for i= 1: size(red_position,2)
    D = norm(red_position(1:3,i)-blue_position(1:3,i))
%     R = norm(red_position_2(1:3,i)-blue_position_2(1:3,i))
    
end

