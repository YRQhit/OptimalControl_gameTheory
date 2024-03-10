% 双椭圆交会 - 大范围转移粗调相角 快速转移克服摄动闭环细调
% 输入：coe_c - 追踪星六根数       coe_t - 目标星六根数    constraint - 约束项
% 输出：机动策略:A星大范围转移 - 飘飞 - 快速转移 - 椭圆变轨（Tjiaohui)
%                B星 - 飘飞（Ttarget） - 椭圆变轨
function [control,result] = tuoYuanJiaoHui(coe_c,coe_t,constraint)
global GM_Earth orbitModel;
startTime = constraint.startTime;    P = constraint.P;
r = coe_t(1);      target = constraint.h;
h = r + constraint.altitude;        T = constraint.T;    r = coe_t(1);
a = (h + r) / 2;
control.dv = (sqrt((2 * h)/(h + r)) - 1) * sqrt(GM_Earth / r);                  % 圆到椭圆速度增量
k = 0.5 - asin((a - r) / a) / pi - (a - r) * sqrt(2*a*r - r^2) / pi / a^2;
q = (1 - k) * 2 * pi * (a / r)^1.5 - pi;
T_later = (q + pi) * sqrt(r^3 / GM_Earth);  
Tjiaohui = (1 - 0.5 * k) * 2 * pi * sqrt(a^3 / GM_Earth);   
Tperoid = pi * (coe_t(1)^3/GM_Earth)^0.5;                                       % 快速转移时间

[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);                     % A星初始位置速度
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);                     % B星初始位置速度
[rv_c0] = [r_c0;v_c0];       [rv_t0] = [r_t0;v_t0];
%% 遍历求得Tp
[~,targetData] = OrbitPrediction(rv_t0, T,60,[1,1],'RK7',startTime);
Mjd = Mjday(startTime(1),startTime(2),startTime(3),startTime(4),startTime(5),startTime(6));
index = [];
for i = 1:length(targetData)
    E = ICRS2ITRS(Mjd);
    [lon,lat,~]= Geodetic(E * targetData(1:3,i));
    if LonLanDiscrimination([lon lat], P, 1000)...
        && 60 * (i - 1) > 2 * Tjiaohui ... 
        index = [index,i];
    end
    Mjd = Mjd + 60 / 86400;
end
index = [index,0];
t1Data = []; t1Sum = 0;num = 1;
for i = 2:length(index)
    if index(i) - index(i - 1) == 1
        t1Sum = t1Sum + index(i - 1);
        num = num + 1;
    else 
        t1Sum = t1Sum + index(i - 1);
        i = i + 1;
        t1Data = [t1Data,t1Sum / num ];
        t1Sum = 0;num = 1;
    end
end
 result.t = (t1Data(1) - 1) * 60;                  % 总时间
 t1 = (result.t - Tjiaohui - Tperoid) * 0.8;       % 大范围转移理论时间
 if t1 > 86400                                     % 大范围转移限定在一天内
     t1 = 86400;
 end

%% 粗调相角
[control.dafanwei,result.dafanwei] = dafanwei(coe_c,coe_t,t1, q * r,startTime);
control.tpiao = result.t - Tjiaohui - result.dafanwei.T - Tperoid;         % 漂移时间
chasePosVel = [r_c0;v_c0];  targetPosVel = [r_t0;v_t0];

orbitModel = 'HPOP';
chasePosVel(4:6) = chasePosVel(4:6) + control.dafanwei.deltv1 * chasePosVel(4:6) / norm(chasePosVel(4:6));
chasePosVel = OrbitPrediction(chasePosVel,control.dafanwei.Tw1,60,[1,1],'RK7',startTime);
chasePosVel(4:6) = chasePosVel(4:6) + control.dafanwei.deltv2 * chasePosVel(4:6) / norm(chasePosVel(4:6));
chasePosVel = OrbitPrediction(chasePosVel,control.dafanwei.T1,60,[1,1],'RK7',AddTime(startTime,control.dafanwei.Tw1));
chasePosVel(4:6) = chasePosVel(4:6) + control.dafanwei.deltv3 * chasePosVel(4:6) / norm(chasePosVel(4:6));
chasePosVel = OrbitPrediction(chasePosVel,control.dafanwei.Tw2,60,[1,1],'RK7',AddTime(startTime,control.dafanwei.Tw1 + control.dafanwei.T1));
chasePosVel(4:6) = chasePosVel(4:6) + control.dafanwei.deltv4;
targetPosVel = OrbitPrediction(targetPosVel,result.dafanwei.T,60,[1,1],'RK7',startTime);

vtemp = banfeicompensate(chasePosVel,targetPosVel) * chasePosVel(4:6) / norm(chasePosVel(4:6));
chasePosVel(4:6) = chasePosVel(4:6) + vtemp;      control.dafanwei.deltv4 = control.dafanwei.deltv4 + vtemp;
chasePosVel = OrbitPrediction(chasePosVel,control.tpiao,60,[1,1],'RK7',AddTime(startTime,result.dafanwei.T));
targetPosVel = OrbitPrediction(targetPosVel,control.tpiao,60,[1,1],'RK7',startTime);

orbitModel = 'LPOP';
coe = State_rv_2_Orbit_Element(chasePosVel(1:3),chasePosVel(4:6));
e = 0;
targetPosVel = OrbitPrediction(targetPosVel,T_later + Tperoid ,60,[1,1],'RK7',startTime);
targetPosVel(4:6) = targetPosVel(4:6) + control.dv * targetPosVel(4:6) / norm(targetPosVel(4:6));

%% 修正相角
deltv1 = [0;0;0];    deltv2 = [0;0;0];    E = [];
rvc = chasePosVel;
for i = 1:30
    control2 = kuaisuzhuanyi(coe,coe,0.1*e,startTime); 
    deltv1 = deltv1 + control2.deltv1;   deltv2 = deltv2 + control2.deltv2;
    rvc(4:6) = rvc(4:6) + deltv1;
    rvc = OrbitPrediction(rvc,Tperoid,60,[1,1],'RK7',startTime);
    rvc(4:6) = rvc(4:6) + deltv2;
    rvc(4:6) = rvc(4:6) + control.dv * rvc(4:6) / norm(rvc(4:6));
    rvc = OrbitPrediction(rvc,T_later,60,[1,1],'RK7',startTime);
    [err, t] = jiaohui(rvc,targetPosVel,Tjiaohui,startTime);
    e = target - err;E = [E e];
    if(abs(e) < 0.01)
        break;
    end
    rvc = chasePosVel;
end
control.kuaisuzhuanyi.deltv1 = deltv1;     control.kuaisuzhuanyi.deltv2 = deltv2;
control.Tjiaohui = t + T_later;            control.kuaisuzhuanyi.T = control2.T;
control.Ttarget = result.dafanwei.T + control.kuaisuzhuanyi.T + control.tpiao +  T_later;
end

%% 确定交会距离和交会时间
function [e,t] = jiaohui(rvc,rvt,T,startTime)
[~,chaseData] = OrbitPrediction(rvc,1.1 * T,60,[1,1],'RK7',startTime);
[~,targetData] = OrbitPrediction(rvt,1.1 * T,60,[1,1],'RK7',startTime);
num = length(chaseData);   distance = zeros(1,num);
index = 1; min = rvc(1);
for i = 1:num
    distance(i) = norm(chaseData(1:3,i) - targetData(1:3,i));
    if distance(i) < min
        min = distance(i); 
        index = i;
    end
end
rvc = chaseData(:,index);   rvt = targetData(:,index);   min = distance(index);
Ttemp = 30;   t = 60 * index - 60;
for i = 1:10
    chasePosVelA = OrbitPrediction(rvc,Ttemp,Ttemp,[1,1],'RK7',startTime);
    targetPosVelA = OrbitPrediction(rvt,Ttemp,Ttemp,[1,1],'RK7',startTime);
    A = norm(chasePosVelA(1:3) - targetPosVelA(1:3));
    
    chasePosVelB = OrbitPrediction(rvc,-Ttemp,Ttemp,[1,1],'RK7',startTime);
    targetPosVelB = OrbitPrediction(rvt,-Ttemp,Ttemp,[1,1],'RK7',startTime);
    B = norm(chasePosVelB(1:3) - targetPosVelB(1:3));
    
    if A < min && B > A 
        min = A;rvc = chasePosVelA;
        rvt = targetPosVelA;
        t = t + Ttemp;
    elseif B < min && A > B
        min = B;rvc = chasePosVelB;
        rvt = targetPosVelB;
        t = t - Ttemp;
    end
    Ttemp = Ttemp - 30 / 2^i;
end
e = JudgePhase(rvc,rvt) * min;
end


