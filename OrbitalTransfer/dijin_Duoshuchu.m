function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw] = dijin_Duoshuchu(coe_c , coe_t , Distance_B ,startTime)
global GM_Earth;

Delta_v_1 = [];
Delta_v_2 = [];
T1 = [];
T2 = [];
Tw = [];
flagg = 1;
JD = Mjday(startTime(1),startTime(2),startTime(3),startTime(4),startTime(5),startTime(6)) + 2400000.5;
T_ff = Tf_Solve(coe_c , coe_t , Distance_B , JD , GM_Earth, flagg);
if size(T_ff) == 0
    flagg = -1;
    T_ff = Tf_Solve(coe_c , coe_t , Distance_B , JD , GM_Earth, flagg);
%     T_f = T_f + 43200
end

T_f = T_ff(1);
j = 0;
for i = 1:length(T_f)
    [delta_v_1 , delta_v_2 , t1 , t2 , tw , flagg , ~] = dijin_Gongmian_Hohmann(coe_c , coe_t , Distance_B , GM_Earth , T_f(i),startTime);
    if (flagg==1 || flagg==2) && t1 > 10
        j = j + 1;
        Delta_v_1 = cat(1 , Delta_v_1 , delta_v_1);
        Delta_v_2 = cat(1 , Delta_v_2 , delta_v_2);
        T1 = cat(1 , T1 , t1);
        T2 = cat(1 , T2 , t2);
        Tw = cat(1 , Tw , tw);
    end
    if j == 10
       return; 
    end
end

end

function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw , flagg , thetaa] = dijin_Gongmian_Hohmann(coe_c , coe_t , Distance_B , muu , T_f,startTime)
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , muu);                    %追踪星初始位置速度矢量(列向量)
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , muu);                    %目标星初始位置速度矢量(列向量)

x_t_0 = [r_t_0 ; v_t_0];                                                  %目标星位置速度(列向量)
if nargin == 6
    yy = OrbitPrediction(x_t_0 ,T_f ,60,[1 1],'RK4',startTime);
elseif nargin == 5
    yy = OrbitPrediction(x_t_0 ,T_f ,60);
end
    
r_t_f = yy (1 : 3);                                                       %目标星最终位置
v_t_f = yy (4 : 6);                                                       %目标星最终速度

c_norm_vector = cross(r_c_0 , v_c_0) / norm(cross(r_c_0 , v_c_0));                         %追踪星平面法向量
if dot(r_t_0 , c_norm_vector) >= 0
    r_t_0_projection2c = r_t_0 - dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%目标星初始位置矢量向追踪星平面投影
else
    r_t_0_projection2c = r_t_0 + dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%目标星初始位置矢量向追踪星平面投影
end
thetaa_0 = acos(dot(r_c_0 , r_t_0_projection2c) / (norm(r_c_0) * norm(r_t_0_projection2c)));            %目标星初始位置矢量投影与追踪星位置矢量夹角(rad)

if nargin == 6
    Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , r_c_0 , v_c_0 , r_t_f , v_t_f , T_f , muu,startTime);
elseif nargin == 5
    Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , r_c_0 , v_c_0 , r_t_f , v_t_f , T_f , muu);
end
a_t_0 = coe_t(1);                           %目标星初始轨道半长轴
a_c_0 = coe_c(1);                           %追踪星初始轨道半长轴
a_c_f = a_t_0 - Distance_B_1;                 %追踪星目标轨道半长轴
a_c_t = (a_c_0 + a_c_f)/2;                  %追踪星转移轨道半长轴

v_c_t_1_before = sqrt(muu / a_c_0);                  %第一次变轨前追踪星速度（按圆轨道计算）
v_c_t_2_after = sqrt(muu / a_c_f);                   %第二次变轨后追踪星速度（按圆轨道计算）
Delta_v_1 = v_c_t_1_before * (sqrt(2 * a_c_f / a_c_0 / (1 +a_c_f / a_c_0)) - 1);        %Hohmann变轨第一次速度脉冲
Delta_v_2 = v_c_t_2_after * (1 - sqrt(2 / (1 + a_c_f / a_c_0)));                        %Hohmann变轨第二次速度脉冲

T_c_t = 2 * pi * sqrt(a_c_t ^ 3 / muu);     %追踪星转移轨道周期

omega_c_0 = 1 / sqrt(a_c_0 ^ 3 / muu);                 %追踪星初始轨道角速度
omega_c_f = 1 / sqrt(a_c_f ^ 3 / muu);                 %追踪星目标轨道角速度
omega_t = 1 / sqrt(a_t_0 ^ 3 / muu);                     %目标星轨道角速度

%------------------优化变量的初始猜测-------------------------%
Tw = T_c_t / 2;                             %轨道转移时间

T2 = (thetaa_0 - pi + omega_t * T_f - (T_f - Tw) * omega_c_0) / (omega_c_f - omega_c_0);

% x_0 = cat(1 , T2);                                      %优化变量初值T2
% 
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% u_lb = cat(1 , 1200);
% u_ub = cat(1 , T_f - Tw);
% p = cat(1 , r_c_0 , v_c_0 , r_t_0 , v_t_0 , T_c_t , Delta_v_1 , Delta_v_2 , r_t_f , v_t_f , T_f);
% 
% [output , fval ,exitflag] = fmincon(@(x) CostFun(x,p) , x_0 , A , b , Aeq , beq , u_lb , u_ub , []);
% 
% T2 = output(1);
% flagg = exitflag;
% thetaa = fval/100000;
T1 = T_f - Tw - T2;
flagg = 1;thetaa = 0;
end

function minJ = CostFun(x,p)
startTime = [2017 1 1 0 0 0];
T2 = x(1);
% p = cat(1 , r_c_0 , v_c_0 , r_t_0 , v_t_0 , T_c_t , Delta_v_1 , Delta_v_2 , r_t_f , v_t_f , T_f);
r_c_0 = p(1 : 3 , :);
v_c_0 = p(4 : 6 , :);
T_c_t = p(13);
Delta_v_1 = p(14);
Delta_v_2 = p(15);
r_t_f = p(16 : 18 , :);
v_t_f = p(19 : 21 , :);
T_f =  p(22);

T1 = T_f - T_c_t / 2 - T2;

x_c_0 = [r_c_0 ; v_c_0];

yy1 = OrbitPrediction(x_c_0 , T1 , 60,[1,1],'RK4',startTime);
r_c_1 = yy1 (1 : 3);
v_c_1 = yy1 (4 : 6);
v_c_1 = v_c_1 / norm(v_c_1) * (norm(v_c_1) + Delta_v_1);
x_c_1 = [r_c_1 ; v_c_1];

yy2 = OrbitPrediction(x_c_1 ,T_c_t / 2 , 60, [1,1], 'RK4',startTime);
r_c_2 = yy2 (1 : 3);
v_c_2 = yy2 (4 : 6);
v_c_2 = v_c_2 / norm(v_c_2) * (norm(v_c_2) + Delta_v_2);
x_c_2 = [r_c_2 ; v_c_2];

yy3 = OrbitPrediction(x_c_2 , T2 , 60,[1,1],'RK4',startTime);
r_c_f = yy3 (1 : 3);
v_c_f = yy3 (4 : 6);

distance = norm(r_c_f - r_t_f)
[~,actualDegree] = IlluminationAngle(yy3,[r_t_f;v_t_f],startTime,T_f)                    % 实际光照角

c_norm_vector = cross(r_c_f , v_c_f) / norm(cross(r_c_f , v_c_f));                         %追踪星平面法向量
if dot(r_t_f , c_norm_vector) >= 0
    r_t_f_projection2c = r_t_f - dot(r_t_f , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;      %目标星末端位置矢量向追踪星平面投影
else
    r_t_f_projection2c = r_t_f + dot(r_t_f , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;      %目标星末端位置矢量向追踪星平面投影
end

thetaa_f = acosd(dot(r_c_f , r_t_f_projection2c) / (norm(r_c_f) * norm(r_t_f_projection2c)))  %目标星末端位置矢量投影与追踪星位置矢量夹角(rad)
minJ = thetaa_f * 100000;

end

function Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , r_c_0 , v_c_0 , r_t_f , v_t_f , T_f , muu,startTime)
x_c_0 = [r_c_0 ; v_c_0];


T_c = 2 * pi * sqrt(coe_c(1) ^ 3 / muu);

if nargin == 10
    yy_c_1 = OrbitPrediction(x_c_0 ,T_f ,60,[1,1],'RK4',startTime);
    yy_c_2 =  OrbitPrediction(yy_c_1 , T_c/2 ,60,[1,1],'RK4',startTime);
elseif nargin == 9
    yy_c_1 = OrbitPrediction(x_c_0 ,T_f ,60);
    yy_c_2 =  OrbitPrediction(yy_c_1 , T_c/2 ,60);
end
mag_c_1 = norm(yy_c_1(1:3));
mag_c_2 = norm(yy_c_2(1:3));

mag_c_d = mag_c_1 - mag_c_2;
mag_t_1 = norm(r_t_f);

x_t_1 = [r_t_f ; v_t_f];
T_t = 2 * pi * sqrt(coe_t(1) ^ 3 / muu);
yy_t_2 =  OrbitPrediction(x_t_1 ,T_t/2 ,60,[1,1],'RK4',startTime);
mag_t_2 = norm(yy_t_2(1:3));

mag_t_d = mag_t_1 - mag_t_2;

Distance_B_1 = Distance_B + (mag_c_d/2) - (mag_t_d/2);

end