function Rrange = fuelestimate(fuelmass,r0,minR)
% RRANGE 用于预估燃料对应的喷气大小，而后利用这个喷气大小判断喷气距离
% fuelmass是燃料质量，r0是初始轨道高度
% 

global GM_Earth


if nargin ==2 
    minR = 6700;
end

dv= fuelmass2dv(fuelmass/4)/1000;       % dv 单位km/s

param = (dv/sqrt(GM_Earth/r0)+1)^2;
r2 = r0*param/(2-param);
dr = r2 - r0;
Rrange = [r0-dr,r2];
if Rrange(1) < minR
    drcomp = minR- Rrange(1);
    Rrange(1) = minR;
    Rrange(2) = r2;
end
end