%%
%配套boyi2
red_sum=0;
blue_sum =0;
red_array=[];
blue_array=[];
red_solution = [];
blue_solution = [];

for i =1:size(solution_array,1)
    if red_sum==blue_sum
        red_sum = red_sum+solution_array(i,4);
        blue_sum = blue_sum+solution_array(i,8);
        red_solution=[red_solution;solution_array(i,1:4)];
        blue_solution=[blue_solution;solution_array(i,5:8)];
    elseif red_sum<blue_sum
        red_sum = red_sum+solution_array(i,4);
        red_solution=[red_solution;solution_array(i,1:4)];
    elseif red_sum>blue_sum
        blue_sum = blue_sum+solution_array(i,8);
        blue_solution=[blue_solution;solution_array(i,5:8)];
    end
    red_array=[red_array,red_sum];
    blue_array=[blue_array,blue_sum];
end



% JD_startTime = datetime([2022,1,1,0,0,0]);
% JD_endTime = datetime([2022,1,1,12,0,0]);
% red = [42100 0 0.1 92 0 280];
% blue = [42166 0 0.1 92 0 280];
% [Red_r,Red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
% [Blue_r,Blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
% 
% for i =0:red_sum
%     
% end
% for j =0:red_sum
%     
% end


%绘制红方和蓝方的位置图
plot3(red_position(1,:),red_position(2,:),red_position(3,:),'->','r')
hold on
plot3(blue_position(1,:),blue_position(2,:),blue_position(3,:),'->','bl')
quiver3(red_position(1,:),red_position(2,:),red_position(3,:))


%%
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


