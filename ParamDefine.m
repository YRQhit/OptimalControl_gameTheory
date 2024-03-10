global GM_Earth GM_Sun GM_Moon r_E CD s_m deg J2 C S orbitModel
global spaceWheather PC rad2deg deg2rad lon_lan;
GM_Earth = 398600.4415;                       % 地球引力参数
GM_Sun = 132712440017.9870;                   % 太阳引力参数
GM_Moon = 4902.79;                            % 月亮引力参数

r_E = 6378.1363;                              % 地球半径
J2 = 1082.6355e-6;                            % 非球形参数

deg = 21;                                     % 默认非球形摄动阶数取20（HPOP)
load('Cnm.mat');    C = Cnm(1:deg + 1,:);     
load('Snm.mat');    S = Snm(1:deg + 1,:);
load('lon_lan.mat'); 

load SpaceWeather.mat;
CD = 2.2;         s_m = 0.02;                % 默认大气摄动参数

load DE405Coeff.mat;
PC = DE405Coeff;                              % 三体摄动星历表

rad2deg = 180 / pi;                        % 弧度转角度
deg2rad = pi / 180;                        % 角度转弧度
orbitModel = 'LPOP';
