% 输入：r1 - 初始位置矢量(建议给出初始速度矢量以确定能量消耗最小的轨道）                
%       r2 - 终止位置矢量             T - 转移时间       
%       startTime - 转移开始时间（给定该参数后使用高精度轨道递推）
% 输出：V1 - 起始速度矢量       V2 - 终止速度矢量     E - 迭代中的误差表
function [V1,V2,E] = lambertIteration(r1,r2,T,startTime)
global GM_Earth;
origin = r1(1:3);         target = r2;           E = [];
errPre = norm(origin - target);

if length(r1) == 6
    v_ref = r1(4:6)';
else
    v_ref = [0 0 0];
end
r1_v1 = cross(origin,v_ref') / norm(cross(origin,v_ref'));

for i = 1:10
     [v1, ~] = lamberthigh(origin, target, T, 0, GM_Earth,'both');             % 二体情况下的兰伯特计算
     if norm(v1(:,:,1) - v_ref') <= norm(v1(:,:,2) - v_ref')                     % 选择与参考速度最接近的速度值
         V1 = v1(:,:,1);
     else
         V1 = v1(:,:,2);
     end
     
     initState = [origin';V1'];
     if nargin <= 3
        finalState = OrbitPrediction(initState,T,60,[1 1],'RK7');                      % 进行轨道递推，输出实际值
     elseif nargin == 4
        finalState = OrbitPrediction(initState,T,60,[1 1 1],'RK7',startTime); 
     end
     
     err = finalState(1:3)' - r2;                                       % 负反馈偏差信号
%      err = DeadZone(r1_v1,err);                                         % 低通滤波器，避免异面兰伯特转移
     e_n = norm(err);                                                   % 偏差模值
     
     if errPre > e_n                                                    % 存储偏差模值最小时的速度
         V1Min = V1;
         V2Min = finalState(4:6)';
         errPre = e_n;
     end
     target = target - err;                                             % 下一次兰伯特计算的目标位置，控制量       
     E = [E e_n];                                                       % 误差表（单位 km） 
     if e_n < 0.01
         break;
     end
end
V1 = V1Min;  V2 = V2Min;
end
%% 低通滤波器
% 输入：r1_v1 - 轨道面的法向量      r2 - 终点矢量
% 输出：r - r2在轨道面上的投影（滤除摄动导致的轨道面漂移）
function r = DeadZone(r1_v1,r2)
angle = dot(r2,r1_v1);
r = r2 - angle * r1_v1;
end








