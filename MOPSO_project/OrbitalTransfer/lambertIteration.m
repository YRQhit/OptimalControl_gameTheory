% ���룺r1 - ��ʼλ��ʸ��(���������ʼ�ٶ�ʸ����ȷ������������С�Ĺ����                
%       r2 - ��ֹλ��ʸ��             T - ת��ʱ��       
%       startTime - ת�ƿ�ʼʱ�䣨�����ò�����ʹ�ø߾��ȹ�����ƣ�
% �����V1 - ��ʼ�ٶ�ʸ��       V2 - ��ֹ�ٶ�ʸ��     E - �����е�����
function [V1,V2,E] = lambertIteration(r1,r2,T,startTime)
global GM_Earth;
origin = r1(1:3);         target = r2;           E = [];
errPre = norm(origin - target);

if length(r1) == 6
    v_ref = r1(4:6)';
else
    v_ref = [0 0 0];
end
r1_v1 = cross(origin,v_ref') / norm(cross(origin,v_ref'));

for i = 1:10
     [v1, ~] = lamberthigh(origin, target, T, 0, GM_Earth,'both');             % ��������µ������ؼ���
     if norm(v1(:,:,1) - v_ref') <= norm(v1(:,:,2) - v_ref')                     % ѡ����ο��ٶ���ӽ����ٶ�ֵ
         V1 = v1(:,:,1);
     else
         V1 = v1(:,:,2);
     end
     
     initState = [origin';V1'];
     if nargin <= 3
        finalState = OrbitPrediction(initState,T,60,[1 1],'RK7');                      % ���й�����ƣ����ʵ��ֵ
     elseif nargin == 4
        finalState = OrbitPrediction(initState,T,60,[1 1 1],'RK7',startTime); 
     end
     
     err = finalState(1:3)' - r2;                                       % ������ƫ���ź�
%      err = DeadZone(r1_v1,err);                                         % ��ͨ�˲�������������������ת��
     e_n = norm(err);                                                   % ƫ��ģֵ
     
     if errPre > e_n                                                    % �洢ƫ��ģֵ��Сʱ���ٶ�
         V1Min = V1;
         V2Min = finalState(4:6)';
         errPre = e_n;
     end
     target = target - err;                                             % ��һ�������ؼ����Ŀ��λ�ã�������       
     E = [E e_n];                                                       % ������λ km�� 
     if e_n < 0.01
         break;
     end
end
V1 = V1Min;  V2 = V2Min;
end
%% ��ͨ�˲���
% ���룺r1_v1 - �����ķ�����      r2 - �յ�ʸ��
% �����r - r2�ڹ�����ϵ�ͶӰ���˳��㶯���µĹ����Ư�ƣ�
function r = DeadZone(r1_v1,r2)
angle = dot(r2,r1_v1);
r = r2 - angle * r1_v1;
end








