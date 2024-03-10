% 伴飞脉冲计算 - 通过修正速度方向的脉冲维持两星距离保持一定
% 输入：rvc - 追踪星位置速度      rvt - 目标星位置速度
% 输出：脉冲策略
function [dv] = banfeicompensate(rvc,rvt)
global GM_Earth;
coe_c = State_rv_2_Orbit_Element(rvc(1:3),rvc(4:6),GM_Earth);
coe_t = State_rv_2_Orbit_Element(rvt(1:3),rvt(4:6),GM_Earth);
d0 = JudgePhase(rvc,rvt) * norm(rvc(1:3) - rvt(1:3));
T_1r = 2 * pi/ omegacal(norm(rvc(1:3)));

rvt_1r = OrbitPrediction(rvt,T_1r,60);
dv = 0;
if coe_c(1) > coe_t(1)
    dir  = -1;
else
    dir  = 1;
end
%% 误差表和误差符号表
e =[];
se = [];

ebest = 100;
% 速度改变量的上下界
maxdv = 20;mindv = 0;
mark =1;            % 过零点标记
dvi_val = 0.0001;    % 初次的速度增量，单位km/s
for i = 1:50
    %% 获得漂飞一轨时间后的两星位置速度，B星就是正常漂飞，放到循环外面
    rvc_1r = OrbitPrediction([rvc(1:3);(rvc(4:6) + dv * rvc(4:6)/norm(rvc(4:6)))],T_1r,60);
    
    %% 近似计算漂飞一轨之后的两星距离差
    val_d1 = norm(rvc_1r(1:3) - rvt_1r(1:3));
    d1 = JudgePhase(rvc_1r,rvt_1r) * val_d1;
    
    % 误差表
    e = [e;d1 - d0];
    se = [se;sign(d1 -d0)];
    if norm(e(i)) < ebest
        ebest = norm(e(i));
        optim.dv = dv;
        optim.e = e(i);
    end
    if norm(e(i)) < 5/1000
        break;
    end
    
    if abs(e(i)) / 10000 > dvi_val
        dv = dv + sign(e(i)) * dvi_val / 5;
    else
        dv = dv + 2e-5 * e(i);
    end
    
    % 给v 一个初始变化
%     if i == 1
%         dv = sign(e(i)) * (norm(dv) + dvi_val);
%     end
%     if i ~= 1
%         if  sign(e(i)) * sign(e(1)) > 0     % 说明 i次修正没有过零点
%             % 保证如果是从第一个开始的连续的没有过0点，累加直到过零点
%             % 同时，如果累加过程中满足了误差条件，则可以通过前面的break跳出
%             mindv = norm(dv);
% %           dv = dir * (norm(dv) + dvi_val/5);
%             if e(i) / 10000 > dvi_val
%                 dv = sign(e(i)) * (norm(dv) + dvi_val / 5);
%             else
%                 dv = sign(e(i)) * (norm(dv) + e(i) / 50000);
%             end
%         end
% 
%         if sign(e(i)) * sign(e(1)) < 0
%             %% 如果这个dv小于 maxdv，则maxdv 是它，在调整的时候阈值在mindv 到maxdv之间
%             if norm(dv) < maxdv
%                 maxdv = norm(dv);
%                 ddv = (maxdv-mindv) / 10;
%             end
%             dv = sign(e(i)) * (norm(dv) - norm(ddv));
%         end
%     end
end
dv = optim.dv;
end