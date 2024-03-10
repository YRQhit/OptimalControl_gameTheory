% 真近点角转平近点角
% 输入：e - 偏心率  trueAno - 真近点角（deg)
% 输出：Me - 平近点角（deg)
function Me = true2Mean(e, trueAno)
global rad2deg deg2rad;
trueAno = trueAno * deg2rad;
E = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(trueAno / 2)); 
Me = (E - e * sin(E)) * rad2deg;
end

