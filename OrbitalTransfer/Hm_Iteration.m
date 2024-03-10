function [delta_v1, delta_v2, theta, Tw]=Hm_Iteration(x1, h_aim, startTime)     
%% 计算过程的前期准备，获得dv1、转移时间Tw以及进入迭代的初始速度v
r1 = norm(x1(1:3));                                                                 %初始的地星连线距
v = x1(4:6);
E = [];
target = h_aim;
%% 开始迭代
for i = 1:10
    [delta_v1, ~, Tw]=Hm_transfer(r1, target);                                       %二体霍曼的两个参数以及初始速度
    v1 = v + v / norm(v) * delta_v1;
    initState = [x1(1:3) ;v1];
    %这部分照搬的lambertIteration，T修改为Tw
    if nargin == 2
        finalState = OrbitPrediction(initState,Tw,60,[1,0],'RK7');                        % 进行轨道递推，输出实际值 
    elseif nargin == 3
        finalState = OrbitPrediction(initState,Tw,60,[1 1],'RK7',startTime); 
    end
    x2 = [finalState(1),finalState(2),finalState(3)];                        %位置向量
    v2 = [finalState(4),finalState(5),finalState(6)];                        %速度向量
    r2 = norm(x2);                                                           %实际的地星距离
    err = r2 - h_aim;        E = [E,err];
    if norm(err) < 0.01                                                       %终止条件
        break;  
    end
    target = target - err;
end
%% 计算航天器转移的相角与dv2
    theta=acosd(dot(x1(1:3),x2)/(norm(x1(1:3))*norm(x2)));                             %向量夹角公式
    v_aim = v_aim_cal(x2,v2);
    delta_v2 = v_aim-v2;
    delta_v2 = delta_v2';
end