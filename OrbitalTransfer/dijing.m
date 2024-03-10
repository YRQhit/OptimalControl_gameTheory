% �ֽ����
% ���룺coe_c - ׷���ǳ�ʼ������      coe_t - Ŀ���ǳ�ʼ������      
%       distance - ׷������Ŀ���ǵ����վ���       modelType - ���ģ�ͣ�����/�߾��ȣ�
% �����control - �����ߣ�
%                 Tw - ת��ʱ��        T1 - Ʈ��ʱ��       deltv - �ܵ��ٶ�����
%                 deltv1��deltv2 - ��һ�α���ٶ�����      
%       result - ���������ݣ�
%                 distance - ����ʱ���Ǿ���     
%                 angle - ����ʱ�������       T - ת������ʱ��������ʱ�����¸�����    
% ex��[control, result] = dijing([7000;0;0;0;0;0], [7500;0;0;0;0;30], 5, 'TwoBody')
%     [control, result] = dijing([7000;0;0;0;0;0], [7500;0;0;0;0;30], 5, 'HPOP')
function [control, result] = dijing(coe_c, coe_t, distance, modelType)
global GM_Earth rad2deg deg2rad;
if nargin == 3
    modelType = 'TwoBody';
end

if strcmp(modelType,'TwoBody')
    model = [0,0];
elseif strcmp(modelType,'HPOP')
    model = [1,1];
end

q1 = AmendDeg(coe_t(6) - coe_c(6));                       % ��ʼĿ���ǳ�ǰ׷���ǵ����               
r0 = coe_c(1);       w0 = sqrt(GM_Earth / r0^3);          % ׷���ǳ�ʼ����뾶����ٶ�            
r1 = coe_t(1);       w1 = sqrt(GM_Earth / r1^3);          % Ŀ���ǹ���뾶����ٶ�
q2 = distance / r1;
q = q1 * deg2rad + q2;                                     % ��׷�����
temp = -sign(q) * (2 * pi - q);                            % ��׷�����

if abs(q) > abs(temp)                                     % ��������²���С�Ƕ�׷��
    q = temp;
end

[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % ׷���ǳ�ʼλ���ٶ�     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % Ŀ���ǳ�ʼλ���ٶ�

[control.deltv1, control.deltv2, control.Tw] = Hm_transfer(r0, r1);       % �����»���ת�ƹ�ʽ
control.T1 = (w1 * control.Tw + q - pi) / (w0 - w1);                      % Ʈ��ʱ���Ľ�����
while control.T1 < 0                                                      % �Ƕ�̫Сʱ�޽⣬��ǲ��2pi
    s = sign(2*pi / (w0 - w1));
    control.T1 = control.T1 + s*2*pi/(w0 - w1);
end

result.T = control.T1 + control.Tw;                                       % ���������ʱ��
result.E = [];
%% �����㶯���µĹ����������������ٶ�������
if strcmp(modelType,'HPOP')                                               
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.T1 ,60,model,'RK7');
    targetPosVel = OrbitPrediction([r_t0;v_t0], result.T ,60,model,'RK7');
    [control.deltv1, control.deltv2, ~, ~] = Hm_Iteration(chasePosVel, norm(targetPosVel(1:3)));
end
control.deltv = abs(norm(control.deltv1)) + abs(norm(control.deltv2));

%% �����㶯���µ���ǲ����Ʈ��ʱ����
for i = 1:30
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.T1 ,60,model,'RK7');
    chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv1;
    chasePosVel = OrbitPrediction(chasePosVel, control.Tw ,60,model,'RK7');  
    chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
        
    targetPosVel = OrbitPrediction([r_t0;v_t0], result.T ,60,model,'RK7');
    e = chasePosVel(1:3) - targetPosVel(1:3);        
    direction = sign(dot(e,chasePosVel(4:6)));
    q2real = direction * acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));
    
    result.distance = norm(e);            result.E = [result.E result.distance];
    result.angle = q2real * rad2deg;
    if strcmp(modelType,'TwoBody')
        break;
    end
    
    if  abs(q2real - q2) * r1 < 0.5
        break;
    else
        eq = q2 - q2real;                                 % �����㷨
        control.T1 = control.T1 + eq / (w0 - w1);
        result.T = control.T1 + control.Tw;
    end  
end
result.coe_c = State_rv_2_Orbit_Element(chasePosVel(1:3),chasePosVel(4:6),GM_Earth);
result.coe_t = State_rv_2_Orbit_Element(targetPosVel(1:3),targetPosVel(4:6),GM_Earth);
result.velDistance = dot(chasePosVel(1:3) - targetPosVel(1:3),targetPosVel(4:6)) / norm(targetPosVel(4:6));
end

