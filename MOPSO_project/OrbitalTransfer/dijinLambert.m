% �ֽ������������ع��ת���㷨
% ���룺coe_c - ׷����������     coe_t - Ŀ����������    Distance_B - �ֽ��㴦׷������Ŀ���Ǿ��루�·�Ϊ����
%       T1 - ׷���ǵ�һ��Ư��ʱ��      T2 - ׷���ǵڶ���Ư��ʱ��      Tw - ׷����ת��ʱ��
% �����d_v1_VVLH��d_v2_VVLH - ���ϵ�����������ٶ�����
% ex:[d_v1_VVLH , d_v2_VVLH] = dijinLambert([6700;0;0;0;0;16] , [6800;0;0;0;0;32] ,-20, 10216.145 , 2016.576 , 2764.555);
function [d_v1_VVLH , d_v2_VVLH] = dijinLambert(coe_c , coe_t , Distance_B, T1 , T2 , Tw)
global GM_Earth;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               %׷���ǳ�ʼλ���ٶ�     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               %Ŀ���ǳ�ʼλ���ٶ�

chasePosVel_1 = OrbitPrediction([r_c0;v_c0],T1,60,[1 1],'RK7');            % ����׷����λ��1
targetPosVel = OrbitPrediction([r_t0;v_t0],T1 + T2 + Tw,60,[1 1],'RK7');

chasePos = targetPosVel(1:3) * (norm(targetPosVel(1:3)) - Distance_B ) / norm(targetPosVel(1:3));         % ׷��������λ��
chaseVel = velocity_cal(chasePos,coe_c(3),coe_c(4));                                                      % ׷���������ٶ�

chasePosVel_2 = OrbitPrediction([chasePos;chaseVel],-T2,60,[1 1],'RK7');   % ����׷����λ��2
[V1,V2,E] = lambertIteration(chasePosVel_1',chasePosVel_2(1:3)',Tw);        % ����������

d_v1 =V1' - chasePosVel_1(4:6);
d_v2 = chasePosVel_2(4:6) - V2';

L_oi_start = Inertial2Orbit(chasePosVel_1 * 1000); 
L_oi_end = Inertial2Orbit(chasePosVel_2 * 1000); 
d_v1_VVLH = L_oi_start * d_v1 * 1000;                                  % ���ϵ�µ�׷���Ǳ��dv1
d_v2_VVLH = L_oi_end * d_v2 * 1000;                                    % ���ϵ�µ�׷���Ǳ��dv2
end

