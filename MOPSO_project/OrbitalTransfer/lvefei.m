% �ӷɣ����ݸ�����Լ���ڶ�����������������Ǳ����ɺ�Ư��ʱ�䡣
% ������ʱ��ν��б������ҳ��������㾭γ�Ⱥ͹��յ�ʱ����Ϊ����
% ��ʱ�䣬���������Ϊת��ǰƮ��ʱ�䡢ת��ʱ���ת�ƺ�Ʈ��ʱ��
% ����������Լ����
% ���룺coe_c - ׷����������     coe_t - Ŀ����������
%    constraint - Լ���ࣨh/T/P/sunDegree/pitch)   startTime - ����ʼʱ��
% �����control - ��������       result - ���ƽ��
function [control,result] = lvefei(coe_c, coe_t, constraint, startTime)
global GM_Earth deg2rad;
h = constraint.h;                      pitch = constraint.pitch;         % Լ����
sunDegree = constraint.sunDegree;       P = constraint.P;       T = constraint.T;

q0 = computeDegree(coe_c, coe_t);               % ���ǳ�ʼ��ǲ�
if q0 * h < 0 || q0 > 2
   disp('�������ӷ�����');
   result.signal = 0;control = [];
   return;
end

[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , GM_Earth); 
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
r = norm(r_c_0); 
[control.delta_v1, control.delta_v2, control.Tw] = Hm_transfer(r, r + h);
q = (90 + pitch) * deg2rad - pi + asin((r + 1) * sind(90 - pitch) / r);                      % ����������ǲ�

w0 = sqrt(GM_Earth / r ^3);
w1 = sqrt(GM_Earth / (r + h) ^3);
t1 = (q - q0 - pi + w0 * control.Tw) / (w1 - w0);                                   % t1�����³�ֵ
targetPosVel = OrbitPrediction([r_t_0 ;v_t_0], control.Tw ,60,[1,1],'RK7',startTime);

%% �������Tp�����㾭γ�ȡ����ս�������
[~,targetData] = OrbitPrediction(targetPosVel, T - t1 - control.Tw ,60,[1,1],'RK7',AddTime(startTime,t1 + control.Tw));
Mjd = Mjday(startTime(1),startTime(2),startTime(3),startTime(4),startTime(5),startTime(6)) + control.Tw / 86400;
index = []; lonlan = zeros(length(targetData),2); sunData = zeros(length(targetData),1);
for i = 1:length(targetData)
    E = ICRS2ITRS(Mjd);
    [lon,lat,~]= Geodetic(E * targetData(1:3,i));  lonlan(i,:) = [lon,lat];  
    sunData(i) = computeSunDegree(targetData(:,i),(pitch + 90), Mjd);
    if LonLanDiscrimination([lon lat], P, 1000)...                       % ����վ��γ��Լ��
        && 60 * (i - 1) > 0.9 * t1... 
        && sunData(i) < sunDegree             % ���ս�Լ��
        index = [index,i]; 
    end
    Mjd = Mjd + 1 / 1440;
end

if isempty(index)
    disp("�Ҳ�������Լ���Ĵ���");
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
     control.t = control.Tw + (t1Data(i) - 1) * 60;                                       % ��һ������
     result.sunDegree = sunData(floor(t1Data(i)));
     result.lonlan = lonlan(floor(t1Data(i)),:);

     [control.Tp, result.degree] = computeTime([r_c_0;v_c_0], [r_t_0;v_t_0], h, control.t, t1, pitch,startTime); 
     if abs(result.degree - pitch) /  pitch < 1
         break;
     end
     control.t1 = control.t - control.Tw - control.Tp;
end
end

%% ʱ������㷨����ʱ��һ�������ת��ʱ�䲻�ɵ���ͨ������ת��ǰ������ʱ�����㸩����Լ����
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

%% ����B��λ�ü���������̫�����ս�
function sunDegree = computeSunDegree(targetPosVel, q, Mjd)
if IsInEarthShadow(targetPosVel(1:3),Mjd) == 1                % �Ƿ��ڵ�Ӱ��
    sunDegree = 180;
    return;
end
n = cross(targetPosVel(4:6),targetPosVel(1:3));
n = n / norm(n); n1 = n(1);n2 = n(2);n3 = n(3);
T = [n1^2*(1 - cosd(q))+cosd(q)        n1*n2*(1 - cosd(q))-n3*sind(q)       n1*n3*(1 - cosd(q))+n2*sind(q);
    n2*n1*(1 - cosd(q))+n3*sind(q)     n2^2*(1 - cosd(q))+cosd(q)           n2*n3*(1 - cosd(q))-n1*sind(q);
    n3*n1*(1 - cosd(q))-n2*sind(q)     n2*n3*(1 - cosd(q))+n1*sind(q)       n3^2*(1 - cosd(q))+cosd(q)];

A2B = -T * targetPosVel(1:3);                                                      % Aָ��Bʸ��
sun_earth_vecotr = -CalAstroBodyPos(Mjd + 2400000.5);                              % ̫��ָ�����ʸ��        
sun_target_vector = targetPosVel(1:3) + sun_earth_vecotr;                          % ̫��ָ��Ŀ����ʸ��
sunDegree = 180 - acosd(dot(A2B,sun_target_vector) / norm(A2B) / norm(sun_target_vector));
end

