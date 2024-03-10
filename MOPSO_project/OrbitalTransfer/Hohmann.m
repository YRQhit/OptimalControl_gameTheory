% 解析法计算抵近的各段时间
function [T1,T2,Tw] = Hohmann(coe_c,coe_t,distance,T_ff,startTime)
global GM_Earth;
for i = 1 : length(T_ff)
    T_f = T_ff(i);

    a_t_0 = coe_t(1);                             %目标星初始轨道半长轴
    a_c_0 = coe_c(1);                             %追踪星初始轨道半长轴
    
    if nargin == 5
        Distance_B_1 = Distance_B_adjust(coe_c , coe_t , distance , T_f ,startTime);
    else
        Distance_B_1 = Distance_B_adjust(coe_c , coe_t , distance , T_f);
    end
    
    a_c_f = a_t_0 - Distance_B_1;                     %追踪星目标轨道半长轴
    a_c_t = (a_c_0 + a_c_f)/2;                    %追踪星转移轨道半长轴

    omega_c_0 = 1 / sqrt(a_c_0 ^ 3 / GM_Earth);                 %追踪星初始轨道角速度
    omega_c_f = 1 / sqrt(a_c_f ^ 3 / GM_Earth);                 %追踪星目标轨道角速度
    omega_t = 1 / sqrt(a_t_0 ^ 3 / GM_Earth);                    %目标星轨道角速度

    if coe_c(2:5) == coe_t(2:5)
        thetaa_0 = (coe_t(6) - coe_c(6)) * pi / 180;
    else
        thetaa_0 = ComputeDegree(coe_c,coe_t);
    end
    Tw = pi * sqrt(a_c_t ^ 3 / GM_Earth);                  %追踪星转移轨道周期
    T2 = (thetaa_0 - pi + omega_t * T_f - (T_f - Tw) * omega_c_0) / (omega_c_f - omega_c_0);
    T1 = T_f - Tw - T2;
    if T2 >= 0 
        break;
    end
end
end
%% 计算目标星与追踪星在追踪轨道面的相角差
function degree = ComputeDegree(coe_c,coe_t)
global GM_Earth;
inclination = coe_c(3);
Rx = [1 0 0;0 cos(inclination) sin(inclination);0 -sin(inclination) cos(inclination)];
c_norm_vector = Rx * [0 0 1]';

[r_c_0 , ~] = Orbit_Element_2_State_rv(coe_c , GM_Earth); 
[r_t_0 , ~] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
if dot(r_t_0 , c_norm_vector) >= 0
    r_t_0_projection2c = r_t_0 - dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector; %目标星初始位置矢量向追踪星平面投影
else
    r_t_0_projection2c = r_t_0 + dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector; %目标星初始位置矢量向追踪星平面投影
end
degree = acos(dot(r_c_0 , r_t_0_projection2c) / (norm(r_c_0) * norm(r_t_0_projection2c)));            %目标星初始位置矢量投影与追踪星位置矢量夹角(rad)
end
%% 距离调整
function Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , T_f ,startTime)
global GM_Earth;
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
x_c_0 = [r_c_0 ; v_c_0];

[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
x_t_0 = [r_t_0 ; v_t_0];

T_c = 2 * pi * sqrt(coe_c(1) ^ 3 / GM_Earth);
T_t = 2 * pi * sqrt(coe_t(1) ^ 3 / GM_Earth);
if nargin == 5
    yy_c_1 = OrbitPrediction(x_c_0 ,T_f ,60,[1,1],'RK4',startTime);
    yy_c_2 =  OrbitPrediction(yy_c_1 , T_c/2 ,60,[1,1],'RK4',startTime);
    
    yy_t_1 = OrbitPrediction(x_t_0 ,T_f ,60,[1,1],'RK4',startTime);
    yy_t_2 = OrbitPrediction(yy_t_1 ,T_t/2 ,60,[1,1],'RK4',startTime);
elseif nargin == 4
    yy_c_1 = OrbitPrediction(x_c_0 ,T_f ,60);
    yy_c_2 =  OrbitPrediction(yy_c_1 , T_c/2 ,60);
    
    yy_t_1 = OrbitPrediction(x_t_0 ,T_f ,60,[1,1],'RK4');
    yy_t_2 = OrbitPrediction(yy_t_1 ,T_t/2 ,60,[1,1],'RK4');
end
mag_c_1 = norm(yy_c_1(1:3));
mag_c_2 = norm(yy_c_2(1:3));
mag_c_d = mag_c_1 - mag_c_2;

mag_t_1 = norm(yy_t_1);
mag_t_2 = norm(yy_t_2(1:3));
mag_t_d = mag_t_1 - mag_t_2;
Distance_B_1 = Distance_B + (mag_c_d/2) - (mag_t_d/2);
end





