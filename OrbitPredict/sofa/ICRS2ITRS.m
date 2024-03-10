%% 地心惯性系(ICRS)到地心固连系(ITRS)的坐标转化矩阵
% 输入：Mjd - 儒略日(UTC)   输出：E - 转换矩阵
function E = ICRS2ITRS(Mjd)
UTC = Mjd;  TT = UTC + 66.184 / 86400;
NPB = iauPnm06a(2400000.5, TT);
gast = iauGst06(2400000.5, UTC, 2400000.5, TT, NPB);
Theta = iauRz(gast, eye(3));
sp = iauSp00(2400000.5, TT);
Pi = iauPom00(0, 0, sp);
E = Pi * Theta * NPB;
end

