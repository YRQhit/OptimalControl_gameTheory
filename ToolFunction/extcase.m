function [T1,Tw1,Tw2] = extcase(r0,r1,r2,q)
% EXTCASE ���ڼ��㼫�������Ҳ����r1 <= 6650km��ʱ����r1λ6650km��T1
% 
[~,~,Tw1] = Hm_transfer(r0,r1);
[~,~,Tw2] = Hm_transfer(r1,r2);
omegat = omegacal(r0);      % ��ʼ�����ƽ���ٶȣ����ܲ������ʽ�ӣ�����û����ν
Tw = Tw1 +Tw2;
omegap = omegacal(r1);
T = (2*pi - omegap * Tw - q)/(omegat - omegap);
T1 = T - Tw;
end