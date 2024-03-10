function [lonlat,E] = rv2lonlat(rv,date)
% RV2LONLAT 是用于将惯性位置速度坐标转化成经纬度的程序
% rv  是惯性坐标系下 卫星的 位置速度 ，6*1,单位为km
% date是一个 6 维向量，分别对应着，年月日时分秒
% 输出：
% lon 经度，单位 度
% lat 纬度，单位 度

% 参考文献 面对快速响应任务的星下点轨迹机动优化问题研究_张海洋 P.16
% 2021 06 01

Mjd = Mjday(date(1), date(2), date(3), date(4), date(5), date(6));
E = ICRS2ITRS(Mjd);
rf = E * rv(1:3);               % 地心固联系下的位置

lon = atand(rf(2)/rf(1));
lat = atand(rf(3)/norm(rf));
lonlat = [lon;lat];
end

