% ����ϵת��γ��
% ���룺Mjd - ������ʱ��     pos - ����ϵ�µ�λ��
% �����lon,lat,h - ��γ��
function [lon,lat,h] = ICRS2LLA(Mjd,pos)
E = ICRS2ITRS(Mjd);                    % ����ϵ������ϵ��ת������
[lon, lat, h] = Geodetic(E * pos);     % ����ϵ����ת��γ��
end

