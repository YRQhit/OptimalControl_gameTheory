% 计算两星相角差
% 输入：coe_c - 追踪星六根数    coe_t - 目标星六根数
% 输出：degree - 两星相角差（追踪星在前为正）
function degree = computeDegree(coe_c,coe_t)
global GM_Earth;
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , GM_Earth); 
[r_t_0 , ~] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
e = r_c_0 - r_t_0; 
degree = sign(dot(e,v_c_0)) * acos(dot(r_c_0,r_t_0) / norm(r_c_0) / norm(r_t_0)); 
end

