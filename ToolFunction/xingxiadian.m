function [T_piao,T_chonghe,lonlat_c_chonghe,dv1,dv2,Tw,Flag_Tpavai] = xingxiadian(rvc,rvt,coe_c,coe_t,dr,lonlat,date)
% XINGXIADIAN 用于计算飘飞多长时间后开始掠飞工况，可以满足星下点经纬度要求
% 执行该工况时，认为追踪星和目标星在同一轨道上只是相差一定的相角
% 核心思路在于
%% 方案2
dfai = AmendDeg(coe_t(6)+coe_t(5)-(coe_c(6)+coe_c(5)));
dfai = dfai*pi/180;
signdfai = sign(dfai);

%% 列出一圈内的经纬度关系对照表
[lonlattable,tardata,T_t] = lonlattablegen(coe_t,rvt,date);                 % T_t 目标星
node1 = 1; 
%% 获得与目标经纬度最近的经纬度
deltalonlattable = lonlat - lonlattable(1:2,:);
for i = 1 : size(deltalonlattable,2)
    deltavaluetable(i) = norm(deltalonlattable(:,i));
end
[~,index] = min(deltavaluetable);
%% 获得从当前时刻起，到目标位置所需要的时间T_need
if index~= size(deltalonlattable,2)
    T_need = 60*(index -1);
else
    T_need = T_tarcircle;
end
node2 =1;
% 对应的位置速度和六要素
rvt_final = tardata(:,index);
coe_t_final = State_rv_2_Orbit_Element(rvt_final(1:3),rvt_final(4:6));
%% 至此和STK都对应过  20210621-21:04
node2 =1 ; % 到此节点，获得了一圈下来的各个准确经纬度以及目标星到达与目标经纬度最近的点所需要花的时间

%% 至此，求出目标星到目标位置所需要的时间，无速度脉冲相关
%% 根据初始的角度关系确定是降轨还是升轨
if dfai > 0
    rlve = norm(rvt(1:3))-dr;
else
    rlve = norm(rvt(1:3))+dr;
end

[delta_v1, delta_v2, ~, Tw]=Hm_Iteration(rvc, rlve, date);
rvc_ellip = rvc + [0;0;0;rvc(4:6)/norm(rvc(4:6))*delta_v1];                  % 经过delta_v1速度脉冲后进入转移轨道
[rvc_beforelve,ChaserData] = OrbitPrediction(rvc_ellip ,Tw ,60,[1,1],'RK7');             % 经过时间Tw后，转移轨道最后的位置速度

%% 追踪星进入掠飞轨道时刻，追踪星和目标星的六要素和位置速度
rvc_lve = rvc_beforelve+[0;0;0;delta_v2];
coe_c_lve = State_rv_2_Orbit_Element(rvc_lve(1:3),rvc_lve(4:6));
[rvt_lve,TargetData] = OrbitPrediction(rvt ,Tw ,60);
coe_t_lve = State_rv_2_Orbit_Element(rvt_lve(1:3),rvt_lve(4:6));

% 此处假设目标星和追踪星的前后关系未改变
dfai2 = AmendDeg(coe_t_lve(6)+coe_t_lve(5)-(coe_c_lve(6)+coe_c_lve(5)));
signdfai2 = sign(dfai2);
dfai2 = signdfai*dfai2*pi/180;      % 这样是的dfai2恒大于零

% 获得追踪星和目标星自轨道转移开始时刻到（我们认为的）两星对地心张角为0时刻所需要的时间
omega_fast = omegacal(min(coe_c_lve(1),coe_t_lve(1)));
omega_slow = omegacal(max(coe_c_lve(1),coe_t_lve(1)));                                 % **新加
Twait = dfai2 / (omega_fast - omega_slow);
T_chonghe = Twait + Tw;
[T_piao,Flag_Tpavai] = tp_guarantee(T_need,T_chonghe,T_t);                             % ***

%% 计算真实的dv1,dv2和重合时 追踪星星下点的经纬度
if Twait >0 && Flag_Tpavai == 1                                                        % ****
    % 飘了一段时间后转移开始时刻的追踪星位置速度rvc_transstart
    rvc_transstart = OrbitPrediction(rvc,T_piao,60);    
    [dv1, dv2, ~, Tw]=Hm_Iteration(rvc_transstart, rlve, date);
    % 实际进入转移轨道的追踪星位置速度
    rvc_ellip_act = rvc_transstart + [0;0;0;rvc_transstart(4:6)/norm(rvc_transstart(4:6))*dv1];                  % 经过delta_v1速度脉冲后进入转移轨道
    [rvc_zhuanyi_act,ChaserData] = OrbitPrediction(rvc_ellip_act ,Tw ,60,[1,1],'RK7');
    %% 实际进入掠飞轨道时刻的追踪星位置速度rvc_lve_actual
    rvc_lve_actual = rvc_zhuanyi_act + [0;0;0;dv2];
    % 实际重合的位置速度
    rvc_chonghe_act = OrbitPrediction(rvc_lve_actual ,Twait ,60,[1,1],'RK7');
    
    %% 计算这个情况（在转移完成后，追踪星和目标星的前后关系没有变化）下的追踪星星下点的最终经纬度
    [lonlat_c,E2] = rv2lonlat(rvc_chonghe_act,AddTime(date,T_need));
    [~,lat,~] = Geodetic(E2*rvc_chonghe_act(1:3));
    lonlat_c_chonghe = [lonlat_c(1);lat];
end                                                                                    % *********
node3 = 1;

if signdfai2*signdfai <0     % 这说明在霍曼转移过程中有两者相角重叠的地方
    for i = 1: size(TargetData,2)
        crs(i) = norm(cross(TargetData(1:3,i),ChaserData(1:3,i)));
    end
    [~,index] = min(crs);
    T_chonghe = 60*(index-1);
    [T_piao,Flag_Tpavai] = tp_guarantee(T_need,T_chonghe,T_t);
    if Flag_Tpavai == 1                                                                 % ********
        %% 求重合时候的rvc
        % 先求转移开始时刻的追踪星位置速度rvc_transstart
        rvc_transstart = OrbitPrediction(rvc,T_piao,60,[1,1],'RK7');
        [dv1, dv2, ~, Tw]=Hm_Iteration(rvc_transstart, rlve, date);
        % 实际进入转移轨道的追踪星位置速度
        rvc_ellip_act = rvc_transstart + [0;0;0;rvc_transstart(4:6)/norm(rvc_transstart(4:6))*dv1];
        %% 再求重合的时候的追踪星位置速度rvc_chonghe
        rvc_chonghe = OrbitPrediction(rvc_ellip_act,T_chonghe,60,[1,1],'RK7');
        [lonlat_c,E2] = rv2lonlat(rvc_chonghe,AddTime(date,T_need));
        [~,lat,~] = Geodetic(E2*rvc_chonghe(1:3));
        lonlat_c_chonghe = [lonlat_c(1);lat];
    end                                                                                 % *******
end

if Flag_Tpavai == 0
    T_chonghe = -1;
    lonlat_c_chonghe = NaN;
    dv1 = 0;
    dv2 = zeros(3,1);
end


%% 0使用大范围转移完成星下点抵达工况
% rlve = norm(rvt(1:3))-dr;
% TA = tae(coe_t_final);
% n =0;
% [control, result] = dafanweizhuanyi(coe_c, [rlve;0;coe_t_final(3:4);0;TA],T_total+ n *T_tarcircle, 0, 'HPOP');
% while result.sign == 0
%     n = n+1;
%     [control, result] = dafanweizhuanyi(coe_c, [rlve;0;coe_t_final(3:6)],T_total+ n *T_tarcircle, 0, 'HPOP');
% end
% % [control, result] = dijin(coe_c, [rlve;0;coe_t_final(3:4);0;TA], 0, 'HPOP');
% node_all = 1;
end
function [T_piao,Flag_avai] = tp_guarantee(T_need,T_chonghe,T_t)
% 用于保证T_piao 的合理性
% T_need 是目标星飘到目标位置所需要的时间
% T_chonghe 是追踪星追上目标星所需要的时间
% T_t 是二体情况下，目标星的轨道周期
% 输出：
% T_piao 是飘飞时间
% Flag_avai 是Tpiao可用标志位，如果为1 则可用，为0 则需要在执行输出T_piao 的基础上再度计算

if T_need >= T_chonghe
    T_piao = T_need - T_chonghe;
    Flag_avai = 1;
else
    T_piao = T_need + 0.5 * T_t;
    Flag_avai = 0;
end
end





