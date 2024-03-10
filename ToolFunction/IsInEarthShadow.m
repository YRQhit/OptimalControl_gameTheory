% 判断卫星是否处在地影内
% 输入：pos - 卫星位置   time - 儒略日
% 输出：1 - 地影内   0 - 地影外
function signal = IsInEarthShadow(pos,time)
global r_E;
earth2sun = CalAstroBodyPos(time + 2400000.5);             % 地球指向太阳矢量
q = asin(r_E / norm(earth2sun));               % 临界角度
d = sqrt(norm(earth2sun)^2 - r_E^2);           % 临界距离

sun2target = earth2sun - pos;                  % 目标星指向太阳矢量
qreal = acos(dot(sun2target,earth2sun) / norm(sun2target) / norm(earth2sun));
if qreal > q || norm(sun2target) < d
    signal = 0;
else
    signal = 1;
end
end

