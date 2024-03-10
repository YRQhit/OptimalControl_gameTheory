% 掠飞：根据俯仰角约束在二体条件下求出任务星变轨完成后漂飞时间。
% 对任务时间段进行遍历，找出满到满足经纬度和光照的时刻作为总任
% 务时间，并将其分配为转移前飘飞时间、转移时间和转移后飘飞时间
% 以满足所有约束。
% 输入：coe_c - 追踪星六根数     coe_t - 目标星六根数
%    constraint - 约束相（h/T/P/sunDegree/pitch)   startTime - 任务开始时间
% 输出：control - 机动策略       result - 控制结果
function [control,result] = lvefei(coe_c, coe_t, constraint, startTime)
global GM_Earth deg2rad;
h = constraint.h;                      pitch = constraint.pitch;         % 约束项
sunDegree = constraint.sunDegree;       P = constraint.P;       T = constraint.T;

q0 = computeDegree(coe_c, coe_t);               % 两星初始相角差
if q0 * h < 0 || q0 > 2
   disp('不满足掠飞条件');
   result.signal = 0;control = [];
   return;
end

[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , GM_Earth); 
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
r = norm(r_c_0); 
[control.delta_v1, control.delta_v2, control.Tw] = Hm_transfer(r, r + h);
q = (90 + pitch) * deg2rad - pi + asin((r + 1) * sind(90 - pitch) / r);                      % 两星最终相角差

w0 = sqrt(GM_Earth / r ^3);
w1 = sqrt(GM_Earth / (r + h) ^3);
t1 = (q - q0 - pi + w0 * control.Tw) / (w1 - w0);                                   % t1二体下初值
targetPosVel = OrbitPrediction([r_t_0 ;v_t_0], control.Tw ,60,[1,1],'RK7',startTime);

%% 遍历求得Tp（满足经纬度、光照角条件）
[~,targetData] = OrbitPrediction(targetPosVel, T - t1 - control.Tw ,60,[1,1],'RK7',AddTime(startTime,t1 + control.Tw));
Mjd = Mjday(startTime(1),startTime(2),startTime(3),startTime(4),startTime(5),startTime(6)) + control.Tw / 86400;
index = []; lonlan = zeros(length(targetData),2); sunData = zeros(length(targetData),1);
for i = 1:length(targetData)
    E = ICRS2ITRS(Mjd);
    [lon,lat,~]= Geodetic(E * targetData(1:3,i));  lonlan(i,:) = [lon,lat];  
    sunData(i) = computeSunDegree(targetData(:,i),(pitch + 90), Mjd);
    if LonLanDiscrimination([lon lat], P, 1000)...                       % 地面站经纬度约束
        && 60 * (i - 1) > 0.9 * t1... 
        && sunData(i) < sunDegree             % 光照角约束
        index = [index,i]; 
    end
    Mjd = Mjd + 1 / 1440;
end

if isempty(index)
    disp("找不到满足约束的窗口");
    result.signal = 0;
    return;
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

for i = 1:length(t1Data)
     control.t = control.Tw + (t1Data(i) - 1) * 60;                                       % 第一个窗口
     result.sunDegree = sunData(floor(t1Data(i)));
     result.lonlan = lonlan(floor(t1Data(i)),:);

     [control.Tp, result.degree] = computeTime([r_c_0;v_c_0], [r_t_0;v_t_0], h, control.t, t1, pitch,startTime); 
     if abs(result.degree - pitch) /  pitch < 1
         break;
     end
     control.t1 = control.t - control.Tw - control.Tp;
end
end

%% 时间分配算法（总时间一定，轨道转移时间不可调，通过调节转移前后两段时间满足俯仰角约束）
function [Tp,degree] = computeTime(rvc, rvt, h, T, t1, pitch,startTime)
[delta_v1, delta_v2, Tw] =  Hm_transfer(norm(rvc(1:3)), norm(rvc(1:3)) + h);
Tp = T - Tw - t1; E = [];   esum = 0;

v = rvc(4:6) + delta_v1 * rvc(4:6) / norm(rvc(4:6));
chasePosVel = OrbitPrediction([rvc(1:3);v],Tw,60,[1,1],'RK7',startTime);
chasePosVel(4:6) = chasePosVel(4:6) + delta_v2 * chasePosVel(4:6) / norm(chasePosVel(4:6));
chasePosVel = OrbitPrediction(chasePosVel,T - Tw,60,[1,1],'RK7',startTime);
targetPosVel = OrbitPrediction(rvt,T,60,[1,1],'RK7',startTime);
relPos = chasePosVel(1:3) - targetPosVel(1:3);
degreeData = acosd(dot(relPos, -chasePosVel(4:6)) / norm(relPos) / norm(chasePosVel(4:6))) - 180 + pitch;
if Tp < 0 || degreeData > 2
    Tp = 0;
    degree = pitch - degreeData;
    return;
end

for i = 1:30
    chasePosVel = OrbitPrediction(rvc,Tp ,60,[1,1],'RK7',startTime);
    chasePosVel(4:6) = chasePosVel(4:6) + delta_v1 * chasePosVel(4:6) / norm(chasePosVel(4:6));
    chasePosVel = OrbitPrediction(chasePosVel,Tw,60,[1,1],'RK7',AddTime(startTime,Tp));
    chasePosVel(4:6) = chasePosVel(4:6) + delta_v2 * chasePosVel(4:6) / norm(chasePosVel(4:6));
    chasePosVel = OrbitPrediction(chasePosVel,t1,60,[1,1],'RK7',startTime);
    relPos = chasePosVel(1:3) - targetPosVel(1:3);
    degreeData = acosd(dot(relPos, -chasePosVel(4:6)) / norm(relPos) / norm(chasePosVel(4:6))) - 180 + pitch;
    
    E = [E,degreeData];
    if abs(degreeData) < 0.5
        break;
    end
    
    if degreeData > 10 || degreeData < -150
        Kp = 100;Ki = 0;
    elseif degreeData < -10 || degreeData > 5
        Kp = 20;Ki = 0;
    elseif degreeData > -10 && degreeData < 5
        Kp = 10;Ki = 0.5;
        esum = esum + degreeData;
    end
    Tp = Tp - sign(h) * (Kp * degreeData + Ki * esum);
    t1 = T - Tw - Tp
    if Tp < 0
        Tp = 0;
        break;
    end
end
degree = pitch - degreeData(end);
plot(E)
end

%% 根据B星位置及俯仰角求太阳光照角
function sunDegree = computeSunDegree(targetPosVel, q, Mjd)
if IsInEarthShadow(targetPosVel(1:3),Mjd) == 1                % 是否处在地影内
    sunDegree = 180;
    return;
end
n = cross(targetPosVel(4:6),targetPosVel(1:3));
n = n / norm(n); n1 = n(1);n2 = n(2);n3 = n(3);
T = [n1^2*(1 - cosd(q))+cosd(q)        n1*n2*(1 - cosd(q))-n3*sind(q)       n1*n3*(1 - cosd(q))+n2*sind(q);
    n2*n1*(1 - cosd(q))+n3*sind(q)     n2^2*(1 - cosd(q))+cosd(q)           n2*n3*(1 - cosd(q))-n1*sind(q);
    n3*n1*(1 - cosd(q))-n2*sind(q)     n2*n3*(1 - cosd(q))+n1*sind(q)       n3^2*(1 - cosd(q))+cosd(q)];

A2B = -T * targetPosVel(1:3);                                                      % A指向B矢量
sun_earth_vecotr = -CalAstroBodyPos(Mjd + 2400000.5);                              % 太阳指向地球矢量        
sun_target_vector = targetPosVel(1:3) + sun_earth_vecotr;                          % 太阳指向目标星矢量
sunDegree = 180 - acosd(dot(A2B,sun_target_vector) / norm(A2B) / norm(sun_target_vector));
end

