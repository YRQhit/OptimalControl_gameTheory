function [dv1,dv2,Tpt,coe_pt] = phasetune(coe_c,coe_t)
% PHASETUNE ���ڵ�����������壩����ʹ��ǰ��������circleorbit.m����ԭ���
% coe_c ��׷������Ҫ�أ�6*1���Ƕȵ�λΪ��
% coe_t ��Ŀ������Ҫ�أ�6*1���Ƕȵ�λΪ��

global GM_Earth
%% �ж�Ŀ���Ǻ�׷�����Ƿ�������������ǰ���ϵ
% �Ƿ���
if abs(coe_c(3)-coe_t(3)) > 0.02 || abs(coe_c(4)-coe_t(4)) > 0.02
    error('��ȷ�����е������ʱ����֤�����������㹲����ֵҪ��');
end

% if abs(coe_c(1)-coe_t(1)) > 10
%     error('��ȷ���������ʱ���볤����С�ڵ���10km');
% end

% if abs(coe_c(2)-coe_c(2)) > 0.005
%     error('��ȷ���������ʱ��ƫ���ʲ����һ����Χ��');
% end

%% �ж��Ƿ��ڽ��ص���е������
% if abs(coe_c(6)) > 0.05
%     error('��ȷ���ڽ��ص�������');
% end
%% �������м���ʱ����ΪĿ���Ǻ�׷���ǹ���
% ������ǲ��Ϊ׷������ǰ��Ϊ��
dfai = (coe_t(6)-coe_c(6)+coe_t(5)-coe_c(5))/180*pi;
n = round(dfai/pi);
dfai = dfai-n*pi;
% ����׷���ǵ�ƽ���ٶȺ�Ŀ���ǵ�ƽ���ٶ�
omegac = sqrt(GM_Earth/(coe_c(1))^3);   % ׷����ƽ���ٶ�
omegat = sqrt(GM_Earth/(coe_t(1))^3);   % Ŀ����ƽ���ٶ�

%% ������������ز���
% ͨ����ǲ��õ�������������Tpt
Tpt = (2 * pi - dfai)/omegat;
apt = (Tpt*sqrt(GM_Earth)/(2*pi))^(2/3);
[rt,vt] = Orbit_Element_2_State_rv(coe_t,GM_Earth);

% Ŀ���ǵĽǶ��������ص㣬Զ�ص�
h = norm(cross(rt,vt));
rat = h^2/GM_Earth * 1/(1-coe_t(2));
% rpt = 2*coe_t(1) - rat;

% �ڽ��ص�ת�ƣ����Խ��ص���ͬ����õ�����������ƫ����
% ׷���ǹ���Ľ��ص㲻�����ŵ������ת�ƹ���Ľ��ص㣬��Ҫ��һ���ж�

r2 = 2 * apt - rat; % ������µ�һ������/Զ���ص�
if r2 < 6378.14 
    warning('���������ཻ');
end

if r2 < rat %˵��ԭ���Ľ��ص��ΪԶ�ص� 
    rapt = r2;
    rppt = rat;
    TApt = 180;
    AOPpt = 180+coe_c(6);
else
    rapt = rat;
    rppt = r2;
    TApt = 0;
    AOPpt = coe_c(6);
end
ept = (rppt-rapt)/(rapt+rppt);

%% ���ڵ�����������ת��ʱ�̵����������������������������ͬ
%% �����Ǻ�������ྭ��׷���ǳ�ʼ�����ͬ
%% ���ص���Ǻ�׷���ǳ�ʼ�����ͬ
if AOPpt > 360
    n = floor(AOPpt/(360));
    AOPpt = AOPpt - 360;
end
coe_pt = [apt;ept;coe_c(3:4);AOPpt;TApt];
[~,vpt] = Orbit_Element_2_State_rv(coe_pt,GM_Earth);
[~,vc] = Orbit_Element_2_State_rv(coe_c,GM_Earth);
dv1 = vpt - vc;
dv2 = vc - vpt;

end

