function [lonlattable] = lonlattablegen(rvdata,date)
% 
% coe_t 是六要素，6*1
% rvt 是位置速度信息
% date 是日期，例如[2015 1 1 0 0 0]

lonlattable =[];
for i = 1 : size(rvdata,2)
    [lonlat_record,E] = rv2lonlat(rvdata(1:3,i),AddTime(date,60*(i-1)));
    [lon,lat,~] = Geodetic(E*rvdata(1:3,i));
    lonlat_T_record= [lon;lat;60*(i-1)];
    lonlattable = [lonlattable,lonlat_T_record];
    if i ~= 1
        if lonlattable(1,i) - lonlattable(1,i-1) > 90
            lonlattable(1,i) = lonlat_record(1) -180;
        elseif lonlattable(1,i) - lonlattable(1,i-1) < -90
            lonlattable(1,i) = lonlat_record(1) +180;
        end
    end
    if lonlattable(1,i)> 180
        lonlattable(1,i) = lonlattable(1,i)-360;
    elseif lonlattable(1,i) < -180
        lonlattable(1,i) = lonlattable(1,i)+360;
    end
end
nodel =1 ;
end

% function [lonlattable,tardata] = lonlattablegen(coe_t,rvt,date)
% %
% % coe_t 是六要素，6*1
% % rvt 是位置速度信息
% % date 是日期，例如[2015 1 1 0 0 0]
% omegat = omegacal(coe_t(1));
% T_tarcircle = 2*pi/omegat;         % 圆轨道的周期，也是认为的目标轨道的周期
% [~,tardata] = OrbitPrediction(rvt,T_tarcircle,60);  % tardata 为目标星递推数据
% 
% lonlattable =[];
% for i = 1 : size(tardata,2)
%     date = AddTime(date,60*(i-1));
%     Mjd = Mjday(date(1), date(2), date(3), date(4), date(5), date(6));
%     E = ICRS2ITRS(Mjd);
%     [lon,lat,~] = Geodetic(E*tardata(1:3,i));
%     lonlat_T_record= [lon;lat;60*(i-1)];
%     lonlattable = [lonlattable,lonlat_T_record];
%     if i ~= 1
%         if lonlattable(1,i) - lonlattable(1,i-1) > 90
%             lonlattable(1,i) = lon -180;
%         elseif lonlattable(1,i) - lonlattable(1,i-1) < -90
%             lonlattable(1,i) = lon +180;
%         end
%     end
%     if lonlattable(1,i)> 180
%         lonlattable(1,i) = lonlattable(1,i)-360;
%     elseif lonlattable(1,i) < -180
%         lonlattable(1,i) = lonlattable(1,i)+360;
%     end
% end
% 
% end