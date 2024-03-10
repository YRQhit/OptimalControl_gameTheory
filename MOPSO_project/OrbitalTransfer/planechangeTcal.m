function [T,Dir_intersection,n2_0] = planechangeTcal(rvc,rvt)
% PLANECHANGETCAL 用于计算从原来的圆轨道转移到异面的圆轨道的时刻（假设xc为0时刻）以及所需要的deltaV
% 不管怎么样的目标轨道，该函数用于转移到轨道高度相同的目标轨道平面。
% rvc 是追踪星的位置速度信息，6*1的向量，单位km
% rvt 是目标星的位置速度信息，6*1的向量，单位km

%%  分离各个参数的r和v
% 追踪星相关参数
rc = rvc(1:3);
vc = rvc(4:6);
rc_val = norm(rc);
vc_val = norm(vc);

% 目标性相关参数
rt = rvt(1:3);
vt = rvt(4:6);

%% 用叉乘计算单位法向量,获得交线矢量
n1_0 = cross(rc,vc)/norm(cross(rc,vc));               % 追踪星最初的轨道平面（由于摄动的原因，其轨道平面必然随着递推而改变）
n2_0 = cross(rt,vt)/norm(cross(rt,vt));               % 目标星最初的轨道平面（由于摄动的原因，其轨道平面必然随着递推而改变）

Dir_intersection = cross(n1_0,n2_0)/norm(cross(n1_0,n2_0));                  % 这是两个轨道平面的交线矢量

%% 有两个理想的转移位置（考虑顺行轨道），给出此时的追踪星的位置速度信息
% rtrans_1 = Dir_intersection * rc_val;
% rtrans_2 = - Dir_intersection * rc_val;

%% 获得rv矢量后，转六要素，获得真近点角信息（但是rv转六要素对于圆轨道，真近点角会奇异，所以不用这个方法）

% rv_trans1 = [rtrans_1;vtrans_1];
% rv_trans2 = [rtrans_2;vtrans_2];

% coe_trans1 = State_rv_2_Orbit_Element(rtrans_1 ,vtrans_1);
% coe_trans2 = State_rv_2_Orbit_Element(rtrans_2 ,vtrans_2);

%% 获得rv矢量后，直接通过角度的定义，由内积来获得角度,arccos 正好是0到pi，此时，定义相角1 为离得近的可转移点对应的相角差
cosXJ1 = dot(rc,Dir_intersection)/(norm(rc)*norm(Dir_intersection));
cosXJ2 = dot(rc,-Dir_intersection)/(norm(rc)*norm(Dir_intersection));


if cosXJ1 > 0           % 意味着相角1 是锐角
    XJ1 = acos(cosXJ1);
elseif cosXJ1 < 0       % 意味着相角1 是一个大于180度的钝角
    XJ1 = acos(cosXJ2); % 这意味着，离得近的那个，叫做相角1
    Dir_intersection = -Dir_intersection;
end
% XJ2 = pi+XJ1;           % 相角2 ，但是默认在相角1 的时候就进行转移，所以相角2 暂时无用，故注释掉

% 计算角速度
omegac = omegacal(norm(rc));

% 理想情况下，应该在T_ideal后进行转移，但是因为有摄动，所以在运行0.95*T_ideal后再计算一次交线作为修正
T = XJ1/omegac;

end

