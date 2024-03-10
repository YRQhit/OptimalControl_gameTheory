% 惯性系转经纬高
% 输入：Mjd - 儒略日时间     pos - 惯性系下的位置
% 输出：lon,lat,h - 经纬高
function [lon,lat,h] = ICRS2LLA(Mjd,pos)
E = ICRS2ITRS(Mjd);                    % 惯性系到固连系的转换矩阵
[lon, lat, h] = Geodetic(E * pos);     % 固连系坐标转经纬高
end

