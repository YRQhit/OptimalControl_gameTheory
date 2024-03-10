% 二体轨道快速计算
% 输入：rv0 - 初始轨道位置速度      T - 递推时间
% 输出：rv - 最终轨道位置速度
function rv = twoBodyOrbitRV(rv0, T)
coe0 = State_rv_2_Orbit_Element(rv0(1:3), rv0(4:6));
if coe0(2) < 0 || coe0(2) > 1
    err('六根数不正确');
end

coe = twoBodyOrbit(coe0,T);
[r,v] = Orbit_Element_2_State_rv(coe);
rv = [r;v];
end

