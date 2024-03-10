global GM_Earth GM_Sun GM_Moon r_E CD s_m deg J2 C S orbitModel
global spaceWheather PC rad2deg deg2rad lon_lan;
GM_Earth = 398600.4415;                       % ������������
GM_Sun = 132712440017.9870;                   % ̫����������
GM_Moon = 4902.79;                            % ������������

r_E = 6378.1363;                              % ����뾶
J2 = 1082.6355e-6;                            % �����β���

deg = 21;                                     % Ĭ�Ϸ������㶯����ȡ20��HPOP)
load('Cnm.mat');    C = Cnm(1:deg + 1,:);     
load('Snm.mat');    S = Snm(1:deg + 1,:);
load('lon_lan.mat'); 

load SpaceWeather.mat;
CD = 2.2;         s_m = 0.02;                % Ĭ�ϴ����㶯����

load DE405Coeff.mat;
PC = DE405Coeff;                              % �����㶯������

rad2deg = 180 / pi;                        % ����ת�Ƕ�
deg2rad = pi / 180;                        % �Ƕ�ת����
orbitModel = 'LPOP';
