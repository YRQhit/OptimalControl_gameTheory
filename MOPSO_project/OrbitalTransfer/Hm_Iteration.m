function [delta_v1, delta_v2, theta, Tw]=Hm_Iteration(x1, h_aim, startTime)     
%% ������̵�ǰ��׼�������dv1��ת��ʱ��Tw�Լ���������ĳ�ʼ�ٶ�v
r1 = norm(x1(1:3));                                                                 %��ʼ�ĵ������߾�
v = x1(4:6);
E = [];
target = h_aim;
%% ��ʼ����
for i = 1:10
    [delta_v1, ~, Tw]=Hm_transfer(r1, target);                                       %������������������Լ���ʼ�ٶ�
    v1 = v + v / norm(v) * delta_v1;
    initState = [x1(1:3) ;v1];
    %�ⲿ���հ��lambertIteration��T�޸�ΪTw
    if nargin == 2
        finalState = OrbitPrediction(initState,Tw,60,[1,0],'RK7');                        % ���й�����ƣ����ʵ��ֵ 
    elseif nargin == 3
        finalState = OrbitPrediction(initState,Tw,60,[1 1],'RK7',startTime); 
    end
    x2 = [finalState(1),finalState(2),finalState(3)];                        %λ������
    v2 = [finalState(4),finalState(5),finalState(6)];                        %�ٶ�����
    r2 = norm(x2);                                                           %ʵ�ʵĵ��Ǿ���
    err = r2 - h_aim;        E = [E,err];
    if norm(err) < 0.01                                                       %��ֹ����
        break;  
    end
    target = target - err;
end
%% ���㺽����ת�Ƶ������dv2
    theta=acosd(dot(x1(1:3),x2)/(norm(x1(1:3))*norm(x2)));                             %�����нǹ�ʽ
    v_aim = v_aim_cal(x2,v2);
    delta_v2 = v_aim-v2;
    delta_v2 = delta_v2';
end