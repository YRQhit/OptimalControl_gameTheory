% ��Χת�����
% ���룺coe_c - ׷���ǳ�ʼ������      coe_t - Ŀ���ǳ�ʼ������       T - ��ʱ��
%       distance - ׷������Ŀ���ǵ����վ���       modelType - ���ģ�ͣ�����/�߾��ȣ�
% �����control - ������ ��
%                 Tw1 - ��һ��ת��ʱ��       Tw2 - �ڶ���ת��ʱ��
%                 T1 - Ʈ��ʱ��              deltv - �ܵ��ٶ�����
%                 deltv1��deltv2 - ��һ�α���ٶ�����          deltv3��deltv4 - �ڶ��α���ٶ�����
%       result - ���������ݣ�
%                 r1 - ת�ƹ���߶�            distance - ����ʱ���Ǿ���
%                 sign - �����Ч��־
%                 angle - ����ʱ�������       T - ת������ʱ��������ʱ�����¸�����
%ex: [control, result] = dafanweizhuanyi([7000;0;0;0;0;0],
%                   [7500;0;0;0;0;30], 20000, 5, 'TwoBody')
%    [control, result] = dafanweizhuanyi([7000;0;0;0;0;0],
%                   [7500;0;0;0;0;30], 20000, 5, 'HPOP')
function [control, result] = dafanweizhuanyi(coe_c, coe_t, T, distance, modelType)
global GM_Earth rad2deg deg2rad;
%% ����ȱʡ
if nargin == 4
    modelType = 'TwoBody';
end

if strcmp(modelType,'TwoBody')
    model = [0,0];
elseif strcmp(modelType,'HPOP')
    model = [1,1];
end

[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % ׷���ǳ�ʼλ���ٶ�
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % Ŀ���ǳ�ʼλ���ٶ�
w2 = sqrt(GM_Earth / coe_t(1)^3);                                         % Ŀ�������ٶ�
control.deltv0 = 0;

%% ƫ���ʹ���ʱ����ΪԲ
if coe_c(2) > 0.0016                                                      % ƫ���ʹ���ʱ������ΪԲ
    n = cross(r_c0,v_c0)/norm(cross(r_c0,v_c0));                          % ��λ������
    v_ep = sqrt(GM_Earth/norm(r_c0));                                     % �ڴ��������ٶ�ֵ
    v_epdir = cross(n,r_c0)/norm(cross(n,r_c0));
    v = v_ep *v_epdir;
    control.deltv0 = v - v_c0;
    v_c0 = v;
end
if coe_c(5) == coe_t(5) && coe_c(4) == coe_t(4) && coe_c(2) == coe_t(2) && coe_t(3) == 0
    r0 = coe_c(1);                                  % ��ʼ����߶�
    r2 = coe_t(1);                                  % Ŀ�����߶�
    
    q1 = AmendDeg(coe_t(6) - coe_c(6));             % ��ʼĿ���ǳ�ǰ׷���ǵ����
    q2 = distance / r2;                             % ����׷���ǳ�ǰĿ���ǵ����
    q  = q1 * deg2rad + q2;                         % ��׷�����
    temp = -sign(q) * (2 * pi - q);                 % ��׷�����
    if abs(q) > abs(temp)                           % ��������²���С�Ƕ�׷��
        q = temp;
    end
else
    r0 = norm(r_c0);
    r2 = norm(r_t0);
    q = acos(dot(r_c0, r_t0) / norm(r_c0) / norm(r_t0));
    if sign(dot(r_c0 - r_t0,v_c0)) > 0              % ׷����Ƿ����ж�
        q = -q;
    end
    q2 = distance / r2;                             % ����׷���ǳ�ǰĿ���ǵ����
end

%% ����ʱ��������
% [result.r1, control.T1, control.Tw1, control.Tw2] = iterationSolve(r0, r2, q, T, T0);
[result.r1 , control.T1, control.Tw1, control.Tw2] = solve(r0, r2, q, T);                    % fsolve���

if control.T1 < 0                     % �������Χת������
    disp("�޷��ڹ涨ʱ�����������");
    result.sign = 0;
    control.deltv = inf;
    return;
else
    result.sign = 1;
end
if result.r1 <= 6650
    [control.T1, control.Tw1, control.Tw2] = extcase(r0,6650,r2,q);
    result.r1 = 6650;
elseif result.r1 > 8300
    [control.T1, control.Tw1, control.Tw2] = extcase(r0,8300,r2,q);
    result.r1 = 8300;
end
% [result.r1, control.T1, control.Tw1, control.Tw2] = iterationSolve(r0, r2, q, T, [control.Tw1 control.Tw2]);   % ��������

%% �������ٶ������������㶯���µĹ������
T1temp = 0;     control.deltv1 = 0;
if strcmp(modelType,'HPOP')
    for i = 1:3                                                          % �ڶ���Ļ����ϼ���������߾���
        qtemp = q - 2 * pi + control.Tw1 * w2 + control.Tw2 * w2;        % Ʈ�ɶ���׷�ϵ����
        w1 = qtemp / control.T1;                                         % Ʈ�ɶ���Ŀ���Ľ��ٶȲ�
        result.r1 = (GM_Earth / (w1 + w2)^2)^(1/3);                      % Ʈ�ɹ���߶�
        
        [deltv1, control.deltv2, ~, control.Tw1] = Hm_Iteration([r_c0;v_c0], result.r1);
        control.deltv1 = control.deltv1 + deltv1;
        v_c0 = v_c0 + v_c0 / norm(v_c0) * deltv1;
        [chasePosVel,~] = OrbitPrediction([r_c0;v_c0], control.Tw1 , 60, model,'RK7');
        chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
        
        [chasePosVel,~] = OrbitPrediction(chasePosVel, control.T1 ,60, model,'RK7');
        [targetPosVel,~] = OrbitPrediction([r_t0;v_t0], control.Tw1 + control.Tw2 + control.T1 ,60, model,'RK7');
        [control.deltv3, control.deltv4, ~, control.Tw2] = Hm_Iteration(chasePosVel, norm(targetPosVel(1:3)));
        
        if abs(control.T1 - T1temp) < 1e-3
            break;                                       % ֹͣ����
        else
            T1temp = control.T1;
        end
    end
elseif strcmp(modelType,'TwoBody')
    [control.deltv1, control.deltv2, ~] = Hm_transfer(r0, result.r1);
    [control.deltv3, control.deltv4, ~] = Hm_transfer(result.r1, r2);
    v_c0 = v_c0 + v_c0 / norm(v_c0) * control.deltv1;                          % ׷���Ǳ����ٶ�
end
w1 = sqrt(GM_Earth / result.r1^3);

%% �����㶯���µ���ǲ�
esum = 0;      result.E = [];       qOptimal = pi;                                     % ������ϵ��
erec = [0;0];                 cnt = 0;        erecflag = 0;
for i = 1:50
    [chasePosVel,~] = OrbitPrediction([r_c0;v_c0], control.Tw1 , 60, model,'RK7');                                % ׷���ǵ�һ��ת�ƺ�λ���ٶ�
    if strcmp(modelType,'HPOP')
        chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
    else
        chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv2;
    end
    
    [chasePosVel,~] = OrbitPrediction(chasePosVel, control.T1 ,60, model,'RK7');                                   % ׷����Ʈ�ɺ�λ���ٶ�
    [targetPosVel,~] = OrbitPrediction([r_t0;v_t0], control.Tw1 + control.Tw2 + control.T1 ,60, model,'RK7');      % Ŀ��������λ���ٶ�
    
    chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv3;
    [chasePosVel,~] = OrbitPrediction(chasePosVel, control.Tw2 ,60, model,'RK7');                                 % ׷���ǵڶ���ת�ƺ�λ���ٶ�
    
    e = chasePosVel(1:3) - targetPosVel(1:3);                                                                      % �������λ��
    angle = acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));      % ���Ǽнǣ�rad
    direction = sign(dot(e,chasePosVel(4:6)));                                                                     % �Ƕȷ���
    qreal = direction * angle;
    
    result.distance = norm(e);                    % ���վ���
    result.E = [result.E result.distance];
    result.angle = qreal * rad2deg;               % �������
    
    if abs(qreal - q2) < abs(qOptimal - q2)       % ���żĴ�
        optimal.T1 = control.T1;             optimal.distance = result.distance;
        optimal.angle = result.angle;        qOptimal = qreal;
        optimal.chasePosVel = chasePosVel;   optimal.targetPosVel = targetPosVel;
    end
    
    if  abs(qreal - q2) * r2 < 0.2                % ����
        break;
    else
        eq = q2 - qreal;                          % �����㷨
        esum = esum + eq;
        if i==1
            erec(1) = eq;
            erec(2) = eq;
        elseif i ==2
            erec(2) = eq;
        else 
            erec(1) = erec(2);
            erec(2) = eq;
        end
%         if esum >2 
%             esum = -1;
%         elseif esum < -2
%             esum =1;
%         end
        if abs(eq) / q2 > 0.3
            Kp = 20;Ki = 0;
            if sign(erec(1)) ~= sign(erec(2)) || erecflag ==1
                cnt = cnt +1;
                Kp=Kp/cnt;Ki = 0.02;
                if Kp < 3
                    Kp =3;
                end
                erecflag =1;
            end
        elseif abs(eq) / q2 > 0.1 
            Kp = 8; Ki = 0.2;
        else
            Kp = 2; Ki = 0.1;
        end
        control.T1 = control.T1 + sign(r2 - result.r1) * (Kp * eq / w1 + Ki * esum / w1);
    end
end

%% ��д������
result.distance = optimal.distance;     result.angle = optimal.angle;
control.T1 = optimal.T1;
result.T = control.T1 + control.Tw1 + control.Tw2;
control.deltv = abs(norm(control.deltv0)) + abs(norm(control.deltv1)) + ...
    abs(norm(control.deltv2)) + abs(norm(control.deltv3)) + abs(norm(control.deltv4));
if strcmp(modelType,'HPOP')
    optimal.chasePosVel(4:6) = optimal.chasePosVel(4:6) + control.deltv4;
else
    optimal.chasePosVel(4:6) = optimal.chasePosVel(4:6) + ...
        optimal.chasePosVel(4:6) / norm(optimal.chasePosVel(4:6)) * control.deltv4;
end
result.finalStateChase = optimal.chasePosVel;
result.finalStateTarget = optimal.targetPosVel;
result.coe_t = State_rv_2_Orbit_Element(result.finalStateTarget(1:3),result.finalStateTarget(4:6));
result.coe_c = State_rv_2_Orbit_Element(result.finalStateChase(1:3),result.finalStateChase(4:6));
result.velDistance = dot(optimal.chasePosVel(1:3) - optimal.targetPosVel(1:3),optimal.targetPosVel(4:6))...
    / norm(optimal.targetPosVel(4:6));
end

%% ������������Լ����ʱ��Լ������,����fsolve�ĳ���
function [r1, T1, Tw1, Tw2] = iterationSolve(r0, r2, q, T, T0)
if nargin == 4
    [~, ~, Tw1] = Hm_transfer(r0, r2);              % ��һ��ת��ʱ����ֵ
    Tw2 = Tw1;                                      % �ڶ���ת��ʱ����ֵ
elseif nargin == 5
    Tw1 = T0(1);
    Tw2 = T0(2);
end
global GM_Earth;

w2 = sqrt(GM_Earth / r2^3);                         % Ŀ���ǹ�����ٶ�
T1 = T - Tw1 - Tw2;                                 % Ʈ��ʱ����ֵ
T1temp = 0;
for i = 1:30
    qtemp = q - 2 * pi + Tw1 * w2 + Tw2 * w2;                % Ʈ�ɶ���׷�ϵ����
    w1 = qtemp / T1;                                         % Ʈ�ɶ���Ŀ���Ľ��ٶȲ�
    r1 = (GM_Earth / (w1 + w2)^2)^(1/3);                     % Ʈ�ɹ���߶�
    
    [~, ~, Tw1] = Hm_transfer(r0, r1);                    % ��һ��ת��ʱ��
    [~, ~, Tw2] = Hm_transfer(r1, r2);                    % �ڶ���ת��ʱ��
    
    T1 = T - Tw1 - Tw2;                                   % Ʈ��ʱ��
    if abs(T1 - T1temp) < 1e-3
        break;                                            % ֹͣ����
    else
        T1temp = T1;
    end
end
end

%% ������Լ����ʱ��Լ�����̣������滻�������еĵ�����
function [r1 , T1, Tw1, Tw2] = solve(r0, r2, q, T)
global GM_Earth;
f =  @(r1)(2 * pi - T * sqrt(GM_Earth / r2^3) - q + ...
    (T - pi * sqrt((r0 + r1)^3 / 8 / GM_Earth) - ...
    pi * sqrt((r1 + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1^3));
r1 = fsolve(f,r0);
[~, ~, Tw1] = Hm_transfer(r0, r1);
[~, ~, Tw2] = Hm_transfer(r1, r2);
T1 = T - Tw1 - Tw2;
end




