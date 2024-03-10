% 推力一定计算燃料消耗率 推力T
function k = kCal(T, Isp)
    g0 = 9.8;
    k = -T/g0/Isp;
end