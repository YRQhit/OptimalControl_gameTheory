%% ��������ܶȣ�NRLMSISE-00ģ�ͣ�
% ���룺PosVel - ����λ���ٶ�        startTime - ��ʼʱ�̣�������ʱ���룩    addMjd - ����������     
% �����dens - �����ܶ�
function dens = ComputeDenstiy_HPOP(PosVel,startTime,addMjd)
global E spaceWheather;                                     
[lon,lat,height]= Geodetic(E * PosVel(1:3));
if height >= 1000000                                   % Ĭ��1000km���ϴ����ܶ�Ϊ0
    dens = 0;
    return
else
    startTime = AddTime(startTime , addMjd * 86400);
end

%% ����һ��ĵڼ���
switch startTime(2)
    case 1 
        dayOfYear = 0 + startTime(3);
    case 2
        dayOfYear = 31 + startTime(3);
    case 3
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28);
    case 4
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31;
    case 5
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30;
    case 6
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31;     
    case 7
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30; 
    case 8
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30 + 31;  
    case 9
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30 + 31 + 31;
    case 10
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30 + 31 + 31 + 30; 
    case 11
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
    case 12
        dayOfYear = 31 + (mod(startTime(1),4) == 0) * (startTime(3) + 29)...
                       + (mod(startTime(1),4) ~= 0) * (startTime(3) + 28) + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30;    
end
second = startTime(4) * 3600 + startTime(5) * 60 + startTime(6);
%% ���ô����ܶȺ���  
num = find((spaceWheather(:,1) == startTime(1)) & (spaceWheather(:,2) == startTime(2)) & (spaceWheather(:,3) == startTime(3)));
ap = spaceWheather(num,4);     f107A = spaceWheather(num,6);     f107 = spaceWheather(num - 1,5);

[~,rho] = atmosnrlmsise00(height, lat, lon, startTime(1), dayOfYear, second,f107A, f107,ap);
dens = rho(6);
end









