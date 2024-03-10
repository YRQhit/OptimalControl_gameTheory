
ParamDefine;
global GM_Earth
tic
format long
coe_Mission = [42166;0.0001;0.001;0;0;20];              
coe_Target =  [42166;0.0001;0.001;0;0;20.1];

startTime = [2022 1 1 0 0 0];                         % 任务开始时间
endTime = [2022 1 2 0 0 0];                           % 任务结束时间
%六根转RV
[Mission_r, Mission_v] = Orbit_Element_2_State_rv(coe_Mission, GM_Earth);
[Target_r, Target_v] = Orbit_Element_2_State_rv(coe_Target, GM_Earth);
%轨道递推
delta_time = endTime - startTime
%转化为秒
s_time = delta_time(1)*365*24*3600+delta_time(2)*30*24*3600+delta_time(3)*24*3600+delta_time(4)*3600+delta_time(5)*60+delta_time(6)
T = 300
t = 60*60
%判断
cube_x = 10;
cube_y = 10;
cube_z = 10;
delta = Mission_r-Target_r
%向上取整
time = 0
for i = 1:ceil(s_time/T)
    i
    if((abs(delta(1))>cube_x)||(abs(delta(2))>cube_y)||(abs(delta(3))>cube_z))
        [deltv1, deltv2] = CW2ImpulseTransfer(coe_Mission, coe_Target, [0;0;0;0;0;0], t)
        Mission_v = Mission_v + deltv1;
        Mission_rv = [Mission_r;Mission_v];
        Mission_rv = twoBodyOrbitRV(Mission_rv, t);
        Mission_v = Mission_rv(4:6);
        Mission_v = Mission_v + deltv2;
        Mission_r = Mission_rv(1:3);
        Mission_rv =[Mission_r;Mission_v];
        i = i+ t/T;
        time = time + 3600;
        Target_rv = [Target_r , Target_v];
        Target_rv = twoBodyOrbitRV(Target_rv, t);
        Target_r = Target_rv(1:3);
        Target_v = Target_rv(4:6);
%         delta = Mission_r-Target_r;
%         coe_Mission =  RV2COE(Mission_rv);
%         coe_Target =  RV2COE(Target_rv);
    else
        Mission_rv = [Mission_r;Mission_v];
        Mission_rv = twoBodyOrbitRV(Mission_rv, T);
        Mission_r = Mission_rv(1:3);
        Mission_v = Mission_rv(4:6);
        time = time +300;
        Target_rv = [Target_r , Target_v];
        Target_rv = twoBodyOrbitRV(Target_rv, T);
        Target_r = Target_rv(1:3);
        Target_v = Target_rv(4:6);
%         delta = Mission_r-Target_r;
%         coe_Mission =  RV2COE(Mission_rv);
%         coe_Target =  RV2COE(Target_rv);
    end
    delta = Mission_r-Target_r;
    coe_Mission =  RV2COE(Mission_rv);
    coe_Target =  RV2COE(Target_rv);
end
