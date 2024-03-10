% 抵近任务中兰伯特轨道转移算法
% 输入：coe_c - 追踪星六根数     coe_t - 目标星六根数    Distance_B - 抵近点处追踪星与目标星距离（下方为正）
%       T1 - 追踪星第一段漂飞时长      T2 - 追踪星第二段漂飞时长      Tw - 追踪星转移时长
% 输出：d_v1_VVLH，d_v2_VVLH - 轨道系下两次喷气速度增量
% ex:[d_v1_VVLH , d_v2_VVLH] = dijinLambert([6700;0;0;0;0;16] , [6800;0;0;0;0;32] ,-20, 10216.145 , 2016.576 , 2764.555);
function [d_v1_VVLH , d_v2_VVLH] = dijinLambert(coe_c , coe_t , Distance_B, T1 , T2 , Tw)
global GM_Earth;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               %追踪星初始位置速度     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               %目标星初始位置速度

chasePosVel_1 = OrbitPrediction([r_c0;v_c0],T1,60,[1 1],'RK7');            % 正推追踪星位置1
targetPosVel = OrbitPrediction([r_t0;v_t0],T1 + T2 + Tw,60,[1 1],'RK7');

chasePos = targetPosVel(1:3) * (norm(targetPosVel(1:3)) - Distance_B ) / norm(targetPosVel(1:3));         % 追踪星拦截位置
chaseVel = velocity_cal(chasePos,coe_c(3),coe_c(4));                                                      % 追踪星拦截速度

chasePosVel_2 = OrbitPrediction([chasePos;chaseVel],-T2,60,[1 1],'RK7');   % 倒推追踪星位置2
[V1,V2,E] = lambertIteration(chasePosVel_1',chasePosVel_2(1:3)',Tw);        % 迭代兰伯特

d_v1 =V1' - chasePosVel_1(4:6);
d_v2 = chasePosVel_2(4:6) - V2';

L_oi_start = Inertial2Orbit(chasePosVel_1 * 1000); 
L_oi_end = Inertial2Orbit(chasePosVel_2 * 1000); 
d_v1_VVLH = L_oi_start * d_v1 * 1000;                                  % 轨道系下的追踪星变轨dv1
d_v2_VVLH = L_oi_end * d_v2 * 1000;                                    % 轨道系下的追踪星变轨dv2
end

