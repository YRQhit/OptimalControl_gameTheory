function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw , flagg , thetaa] = dijin_yimian_Main(coe_c, coe_t, Distance_B)
global GM_Earth;

[t_0, t_f] = T0_Tf_Solve(coe_c , coe_t , Distance_B , GM_Earth);
Delta_v_1 = [];
Delta_v_2 = [];
T1 = [];
T2 = [];
Tw = [];
flagg = [];
thetaa = [];

T_f = TimeSolve_1(t_0 , coe_t);
j = 0;
for i = 1 : length(T_f)
    if T_f(i) > t_0 && T_f(i) < t_f
        [delta_v_1 , delta_v_2, t1, t2, tw, flagg, thetaa] = dijin_yimian_Hohmann(coe_c, coe_t, Distance_B, T_f(i), GM_Earth);
        if (flagg == 1 || flagg == 2) && t1>100
            Delta_v_1 = cat(1, Delta_v_1, delta_v_1);
            Delta_v_2 = cat(1, Delta_v_2, delta_v_2);
            T1 = cat(1, T1, t1);
            T2 = cat(1, T2, t2);
            Tw = cat(1, Tw, tw);
            j = j+1;
        end
        if j==10
           break; 
        end
    end
end


end


function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw , flagg , thetaa] = dijin_yimian_Hohmann(coe_c , coe_t , Distance_B , T_f , muu)
startTime = [2017 1 1 0 0 0];
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , muu);                    %׷���ǳ�ʼλ���ٶ�ʸ��(������)
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , muu);                    %Ŀ���ǳ�ʼλ���ٶ�ʸ��(������)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_t_0 = [r_t_0 ; v_t_0];                                                  %������
yy = OrbitPrediction(x_t_0 ,T_f ,60,[1 1],'RK4',startTime);
r_t_f = yy (1 : 3);
v_t_f = yy (4 : 6);

c_norm_vector = cross(r_c_0 , v_c_0) / norm(cross(r_c_0 , v_c_0));                         %׷����ƽ�淨����
if dot(r_t_0 , c_norm_vector) >= 0
    r_t_0_projection2c = r_t_0 - dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%Ŀ���ǳ�ʼλ��ʸ����׷����ƽ��ͶӰ
else
    r_t_0_projection2c = r_t_0 + dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%Ŀ���ǳ�ʼλ��ʸ����׷����ƽ��ͶӰ
end
thetaa_0 = acos(dot(r_c_0 , r_t_0_projection2c) / (norm(r_c_0) * norm(r_t_0_projection2c)));            %Ŀ���ǳ�ʼλ��ʸ��ͶӰ��׷����λ��ʸ���н�(rad)

Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , r_c_0 , v_c_0 , r_t_f , v_t_f , T_f , muu);
a_t_0 = coe_t(1);                           %Ŀ���ǳ�ʼ����볤��
a_c_0 = coe_c(1);                           %׷���ǳ�ʼ����볤��
a_c_f = a_t_0 - Distance_B_1;                 %׷����Ŀ�����볤��
a_c_t = (a_c_0 + a_c_f)/2;                  %׷����ת�ƹ���볤��

v_c_t_1_before = sqrt(muu / a_c_0);                  %��һ�α��ǰ׷�����ٶȣ���Բ������㣩
v_c_t_2_after = sqrt(muu / a_c_f);                   %�ڶ��α���׷�����ٶȣ���Բ������㣩
Delta_v_1 = v_c_t_1_before * (sqrt(2 * a_c_f / a_c_0 / (1 + a_c_f / a_c_0)) - 1);        %Hohmann����һ���ٶ�����
Delta_v_2 = v_c_t_2_after * (1 - sqrt(2 / (1 + a_c_f / a_c_0)));                        %Hohmann���ڶ����ٶ�����

T_c_t = 2 * pi * sqrt(a_c_t ^ 3 / muu);     %׷����ת�ƹ������

omega_c_0 = 1 / sqrt(a_c_0 ^ 3 / muu);                 %׷���ǳ�ʼ������ٶ�
omega_c_f = 1 / sqrt(a_c_f ^ 3 / muu);                 %׷����Ŀ�������ٶ�
omega_t = 1 / sqrt(a_t_0 ^ 3 / muu);                     %Ŀ���ǹ�����ٶ�

%------------------�Ż������ĳ�ʼ�²�-------------------------%
Tw = T_c_t / 2;                             %���ת��ʱ��

T2 = (thetaa_0 - pi + omega_t * T_f - (T_f - Tw) * omega_c_0) / (omega_c_f - omega_c_0);

% x_0 = cat(1 , T2);                                      %�Ż�������ֵT2
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
T1 = T_f - Tw - T2;
flagg = 1; thetaa = 0;
end


function minJ = CostFun(x,p)
startTime = [2017 1 1 0 0 0];
T2 = x(1);

r_c_0 = p(1 : 3 , :);
v_c_0 = p(4 : 6 , :);
T_c_t = p(13);
Delta_v_1 = p(14);
Delta_v_2 = p(15);
r_t_f = p(16 : 18 , :);
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

c_norm_vector = cross(r_c_f , v_c_f) / (norm(r_c_f) * norm(v_c_f));             %׷����ƽ�淨����
if dot(r_t_f , c_norm_vector) >= 0
    r_t_f_projection2c = r_t_f - dot(r_t_f , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;      %Ŀ����ĩ��λ��ʸ����׷����ƽ��ͶӰ
else
    r_t_f_projection2c = r_t_f + dot(r_t_f , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;      %Ŀ����ĩ��λ��ʸ����׷����ƽ��ͶӰ
end
thetaa_f = acosd(dot(r_c_f , r_t_f_projection2c) / (norm(r_c_f) * norm(r_t_f_projection2c)))             %Ŀ����ĩ��λ��ʸ��ͶӰ��׷����λ��ʸ���н�(deg)

minJ = thetaa_f * 10000;
end

function Distance_B_1 = Distance_B_adjust(coe_c , coe_t , Distance_B , r_c_0 , v_c_0 , r_t_f , v_t_f , T_f , muu)
startTime = [2017 1 1 0 0 0];
x_c_0 = [r_c_0 ; v_c_0];
yy_c_1 = OrbitPrediction(x_c_0 ,T_f ,60,[1,1],'RK4',startTime);
mag_c_1 = norm(yy_c_1(1:3));

T_c = 2 * pi * sqrt(coe_c(1) ^ 3 / muu);
yy_c_2 =  OrbitPrediction(yy_c_1 , T_c/2 ,60,[1,1],'RK4',startTime);
mag_c_2 = norm(yy_c_2(1:3));

mag_c_d = mag_c_1 - mag_c_2;

mag_t_1 = norm(r_t_f);

x_t_1 = [r_t_f ; v_t_f];
T_t = 2 * pi * sqrt(coe_t(1) ^ 3 / muu);
yy_t_2 =  OrbitPrediction(x_t_1 ,T_t/2 ,60);
mag_t_2 = norm(yy_t_2(1:3));

mag_t_d = mag_t_1 - mag_t_2;

Distance_B_1 = Distance_B + (mag_c_d/2) - (mag_t_d/2);

end