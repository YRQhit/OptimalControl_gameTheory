function [dv1,dv2,Tpt,coe_pt] = phasetune(coe_c,coe_t)
% PHASETUNE 用于调相机动（二体），在使用前建议先用circleorbit.m规整原轨道
% coe_c 是追踪星六要素，6*1，角度单位为度
% coe_t 是目标星六要素，6*1，角度单位为度

global GM_Earth
%% 判断目标星和追踪星是否满足调相机动的前提关系
% 是否共面
if abs(coe_c(3)-coe_t(3)) > 0.02 || abs(coe_c(4)-coe_t(4)) > 0.02
    error('请确保进行调相机动时，保证两颗卫星满足共面阈值要求');
end

% if abs(coe_c(1)-coe_t(1)) > 10
%     error('请确保调相机动时，半长轴差距小于等于10km');
% end

% if abs(coe_c(2)-coe_c(2)) > 0.005
%     error('请确保调相机动时，偏心率差距在一定范围内');
% end

%% 判断是否在近地点进行调相机动
% if abs(coe_c(6)) > 0.05
%     error('请确保在近地点调相机动');
% end
%% 进行下列计算时，认为目标星和追踪星共面
% 计算相角差，认为追踪星在前面为正
dfai = (coe_t(6)-coe_c(6)+coe_t(5)-coe_c(5))/180*pi;
n = round(dfai/pi);
dfai = dfai-n*pi;
% 计算追踪星的平角速度和目标星的平角速度
omegac = sqrt(GM_Earth/(coe_c(1))^3);   % 追踪星平角速度
omegat = sqrt(GM_Earth/(coe_t(1))^3);   % 目标星平角速度

%% 计算调相机动相关参数
% 通过相角差获得调相机动轨道周期Tpt
Tpt = (2 * pi - dfai)/omegat;
apt = (Tpt*sqrt(GM_Earth)/(2*pi))^(2/3);
[rt,vt] = Orbit_Element_2_State_rv(coe_t,GM_Earth);

% 目标星的角动量，近地点，远地点
h = norm(cross(rt,vt));
rat = h^2/GM_Earth * 1/(1-coe_t(2));
% rpt = 2*coe_t(1) - rat;

% 在近地点转移，所以近地点相同，获得调相机动轨道的偏心率
% 追踪星轨道的近地点不代表着调相机动转移轨道的近地点，需要做一个判断

r2 = 2 * apt - rat; % 算出的新的一个（近/远）地点
if r2 < 6378.14 
    warning('轨道与地球相交');
end

if r2 < rat %说明原来的近地点变为远地点 
    rapt = r2;
    rppt = rat;
    TApt = 180;
    AOPpt = 180+coe_c(6);
else
    rapt = rat;
    rppt = r2;
    TApt = 0;
    AOPpt = coe_c(6);
end
ept = (rppt-rapt)/(rapt+rppt);

%% 对于调相机动轨道，转移时刻的真近点角与调相机动轨道的真近点角相同
%% 轨道倾角和升交点赤经和追踪星初始轨道相同
%% 近地点幅角和追踪星初始轨道相同
if AOPpt > 360
    n = floor(AOPpt/(360));
    AOPpt = AOPpt - 360;
end
coe_pt = [apt;ept;coe_c(3:4);AOPpt;TApt];
[~,vpt] = Orbit_Element_2_State_rv(coe_pt,GM_Earth);
[~,vc] = Orbit_Element_2_State_rv(coe_c,GM_Earth);
dv1 = vpt - vc;
dv2 = vc - vpt;

end

