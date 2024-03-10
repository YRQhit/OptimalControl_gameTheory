% J2轨道快速计算(忽略田谐项)
% 输入 rv0 - 初始rv     T - 递推时间
% 输出 rv - 最终rv
function rv = J2OrbitRV(rv0, T)
global GM_Earth
coe0 = State_rv_2_Orbit_Element(rv0(1:3), rv0(4:6));
if coe0(2) < 0 || coe0(2) > 1
    err('鍏牴鏁颁笉姝ｇ‘');
end

coe = J2Orbit(coe0,T);
[r,v] = Orbit_Element_2_State_rv(coe, GM_Earth);
rv = [r;v];
end