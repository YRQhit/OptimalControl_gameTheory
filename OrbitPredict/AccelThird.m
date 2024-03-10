% 计算三体摄动加速度
% 输入：Pos - 卫星在地心惯性系下的坐标          Mjd - 儒略日
% 输出：a - 三体摄动加速度
function a = AccelThird(Pos , Mjd)
global GM_Sun  GM_Moon;  

[~,~,~,~,~,~,~,~,~,r_moon,r_sun] = JPL_Eph_DE405(Mjd + 66.184 / 86400);
r_moon = r_moon./1000;       r_sun = r_sun./1000;                          % 地心惯性系下的日月位置矢量

erthToSun = (r_sun(1)^2 + r_sun(2)^2 + r_sun(3)^2)^1.5;                    % 日地距离(广义）
earthToMoon = (r_moon(1)^2 + r_moon(2)^2 + r_moon(3)^2)^1.5;               % 月地距离(广义）
 
R_sun = ((Pos(1) - r_sun(1))^2 + (Pos(2) - r_sun(2))^2 + (Pos(3) - r_sun(3))^2)^1.5;           % 卫星与太阳距离
R_moon = ((Pos(1) - r_moon(1))^2 + (Pos(2) - r_moon(2))^2 + (Pos(3) - r_moon(3))^2)^1.5;       % 卫星与月球距离

a_sun = - GM_Sun * ((Pos -  r_sun) / R_sun + r_sun / erthToSun);           % 太阳摄动加速度
a_moon = - GM_Moon * ((Pos -  r_moon) / R_moon + r_moon / earthToMoon);    % 月球摄动加速度
a = a_sun + a_moon;
end

