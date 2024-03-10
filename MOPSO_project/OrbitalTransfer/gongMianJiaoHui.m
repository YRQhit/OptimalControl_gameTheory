% ����˫��Բ����
% ���룺coe_c - A��������          coe_t - B��������       type - ���᷽ʽ
% �����control - �����
%           deltv - �ٶ�����            T - B��Ʈ��ʱ��             TJiaoHui - ����ʱ��
%       result - ��������
%           distance - ������̾���     relativeVel - ��������ٶ�   T2 - ��������
% ex:[control, result] = gongMianJiaoHui([6700;0;0;0;0;2], [6700;0;0;0;0;0], 'short')
function [control, result] = gongMianJiaoHui(coe_c, coe_t, type)
global GM_Earth;
r = coe_c(1);           q = coe_c(6) - coe_t(6);
if q < 0
    q = q + 360;
end

if nargin == 2
    type = 'both';
end

if strcmp(type, 'short')
    [q_a, c] = TrajectoryPlan(r, q, 'short');
elseif strcmp(type, 'long')
    [q_a, c] = TrajectoryPlan(r, q, 'long');

elseif strcmp(type, 'both')
      [control(1), result(1)] = gongMianJiaoHui(coe_c, coe_t, 'short');
      [control(2), result(2)] = gongMianJiaoHui(coe_c, coe_t, 'long');
      return;
end

control.T = (q + 180) / 360 * 2 * pi * sqrt(r^3 / GM_Earth);                % B����Բ�����Ʈ��ʱ��
control.deltv = (sqrt(1 + c / q_a) - 1) * sqrt(GM_Earth / r) * 1000;        % ��������ٶ�����,��λm
T1 = 2 * pi * sqrt(r^3 / GM_Earth);                                         % Բ�������
result.T2 = 2 * pi * sqrt(q_a^3 / GM_Earth);                                % ��Բ�������
k = 0.25 - asin((q_a - r) / q_a) / 2 / pi - (q_a - r) * sqrt(2 * q_a * r - r^2) / 2 / pi / q_a^2;
T3 = k * result.T2;                                                         % B������Բ�����Ʈ��ʱ��

if strcmp(type, 'short')
    control.TJiaoHui = result.T2 - T3;                                              % ���ǽ���ʱ�䣨����������
elseif strcmp(type, 'long')
    control.TJiaoHui = result.T2 + T3;  
end
    
%% ������̾���,����ٶ�
coe_t = coe_c;     coe_t(6) = coe_c(6) - q;
coe_c(1) = q_a;    coe_c(2) = c / q_a;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % A�ǳ�ʼλ���ٶ�     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % B�ǳ�ʼλ���ٶ�

while 1
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.TJiaoHui ,60,[0 0],'RK7');       % �����A�ǵ�λ���ٶ�
    targetPosVel0 = OrbitPrediction([r_t0;v_t0], control.T ,60,[0 0],'RK7');
    targetPosVel0(4:6) = targetPosVel0(4:6) + targetPosVel0(4:6) / norm(targetPosVel0(4:6)) * control.deltv / 1000;
    targetPosVel = OrbitPrediction(targetPosVel0, control.TJiaoHui - control.T ,60,[0 0],'RK7');   % �����B��λ���ٶ�

    result.distance = norm(chasePosVel(1:3) - targetPosVel(1:3));                   % ���ǽ�����̾���
    result.relativeVel = norm(chasePosVel(4:6) - targetPosVel(4:6));                % ���ǽ�������ٶ�
    
    if result.distance > 0.1                                                        % ��ֹ������ײ�����ڼ��������Ǿ���ͨ������Ϊ0��
       break; 
    else
        control.T = control.T + 0.1 * T1 / (r * 2 * pi); 
    end
end
end
%% ���ݳ�ʼ����Ǽ�����뾶���㽻����Բ
% ���룺r - Բ����뾶          q - ��ʼʱ��B���ͺ�A�ǵ����    type - �����ڽ���/�����ڽ��� 
% �����q_a - ������Բ�볤��    c - ������Բ����
function [q_a, c] = TrajectoryPlan(r, q, type)
a = r:5:r + 100;
if strcmp(type, 'short')
    y = ((pi + 2 .* asin((a - r) ./ a) + 2 * (a - r) .* sqrt(2 .* a * r - r^2) ./ a.^2)...
        .* (a ./ r).^1.5 - pi) * 180 / pi;                               % ���ݹ������ѧ�õ��İ볤������ǹ�ϵ�������ڽ��ᣩ
elseif strcmp(type, 'long')
    y = ((pi - 2 * asin((a - r) ./ a) - 2 * (a - r) .* sqrt(2 .* a * r - r^2) ./ a.^2)... 
        .* (a ./ r).^1.5 - pi) * 180 / pi;                                % ���ݹ������ѧ�õ��İ볤������ǹ�ϵ�������ڽ��ᣩ
end

x = [ones(length(a),1) a'];
param = pinv(x) * y';                                               % һ�κ�����Ϲ�ϵ                             
q_a = (q - param(1)) / param(2);                                    % ����Ϲ�ϵ�õ�����Բ�볤��  
while 1
    if strcmp(type, 'short')
        y_q = ((pi + 2 * asin((q_a - r) / q_a) + 2 * (q_a - r) * sqrt(2 * q_a * r - r^2) / q_a^2)...
            * (q_a / r)^1.5 - pi) * 180 / pi;   
    elseif strcmp(type, 'long')
        y_q = ((pi - 2 * asin((q_a - r) / q_a) - 2 * (q_a - r) * sqrt(2 * q_a * r - r^2) / q_a^2)...
            * (q_a / r)^1.5 - pi) * 180 / pi;
    end
        
    e = y_q - q; 
    if e * r * (pi /180) < 5
        break;
    else
        q_a = q_a + (e - param(1)) / param(2);
    end
end
c = q_a - r;
end


















