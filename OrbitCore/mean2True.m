% 平近点角转真近点角
% 输入：e - 偏心率       Me - 平近点角（deg)
% 输出：trueAng - 真近点角（deg)
function trueAng = mean2True(e,Me)
global rad2deg deg2rad;
Me_r = AmendDeg(Me,'0 - 360') * deg2rad;
E_r  = Me_r;
if Me_r < pi
    E_r = E_r + e/ 2;
else
    E_r = E_r - e/ 2;
end

ration = 1;j = 0;
while abs(ration) > eps
    ration = (E_r - e * sin(E_r) - Me_r) / (1 - e * cos(E_r));
    E_r = E_r - ration;
    j = j + 1;
    if j >= 50
        break;
    end
end
trueAng = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E_r / 2)) * rad2deg;
trueAng = AmendDeg(trueAng,'0 - 360');
end

