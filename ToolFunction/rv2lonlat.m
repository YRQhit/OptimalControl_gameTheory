function [lonlat,E] = rv2lonlat(rv,date)
% RV2LONLAT �����ڽ�����λ���ٶ�����ת���ɾ�γ�ȵĳ���
% rv  �ǹ�������ϵ�� ���ǵ� λ���ٶ� ��6*1,��λΪkm
% date��һ�� 6 ά�������ֱ��Ӧ�ţ�������ʱ����
% �����
% lon ���ȣ���λ ��
% lat γ�ȣ���λ ��

% �ο����� ��Կ�����Ӧ��������µ�켣�����Ż������о�_�ź��� P.16
% 2021 06 01

Mjd = Mjday(date(1), date(2), date(3), date(4), date(5), date(6));
E = ICRS2ITRS(Mjd);
rf = E * rv(1:3);               % ���Ĺ���ϵ�µ�λ��

lon = atand(rf(2)/rf(1));
lat = atand(rf(3)/norm(rf));
lonlat = [lon;lat];
end

