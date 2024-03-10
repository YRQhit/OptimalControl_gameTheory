function [T,index] = orbitalterTime(rv_current,date,Tremain)
% ORBITALTERPOINT的作用是确定圆轨道的变轨点
% rv_current 是追踪星当前速度
% date 是当前时刻
% Tremain 是任务剩余时间
% 输出
% T 自满足相角条件开始，到转移开始的时间
global lon_lan

coe_current = State_rv_2_Orbit_Element(rv_current(1:3),rv_current(4:6));
% 默认没找到
index = -1;

%% 生成n圈的经纬度信息（默认两圈）
omegat = omegacal(coe_current(1));

T_1r= 2*pi/omegat;    % 这样几乎可以确定跑了两圈多
n = floor(Tremain / T_1r);
if n < 0 
    error('任务剩余时间错误');
end
[rv_2c,tardata] = OrbitPrediction(rv_current,Tremain,60);   % rv_2c 是两圈后的rv
[lonlattable] = lonlattablegen(tardata,date);

%% 遍历lonlattable，获得Inchina指示标志
for i = 1: size(lonlattable,2)
    inChina = LonLanDiscrimination(lonlattable(1:2,i));    % 第i列的经纬度
    if inChina == 1
        index =i;
        %% 保证飘飞时间为正
        if (60*(i -1)- 0.75*T_1r) < 0
            index = -1;
            continue;
        else
            break;
        end
        %% 源程序
        %         inChina2 = LonLanDiscrimination(lonlattable(:,i+1));
        %         if (inChina+inChina2) == 2
        %             index = i;
        %             break;
        %         end
        
    end
end
if index>0
    rv = tardata(:,index);  %此处为与交会点共线的初始轨道上的rv
    T = 60*(i-1)-0.75*T_1r;
end

if inChina == 0
    msgbox('任务指定时间内无法过境中国','双椭圆交会信息');
end
end