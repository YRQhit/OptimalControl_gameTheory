% 角度修正函数，根据不同要求的角度范围调整实际角度值
% 输入：deg0 - 原始角度      type - 角度范围
% 输出：deg - 满足范围要求的角度值
% 补充 - STK中相关角度的范围：
%     轨道倾角："0 - 180"            近地点幅角："0 - 360"      
%     升交点赤经："-180 - 360"       真近点角："-180 - 360" 
function deg = AmendDeg(deg0, type)
if nargin == 1
    type = "-180 - 360";
end

if strcmp(type,"0 - 180")
    range = [0 180];
elseif strcmp(type,"0 - 360")
    range = [0 360];
elseif strcmp(type,"-180 - 360")
    range = [-180 360];
else
    disp("该范围有误");
    return;
end    

if deg0 > range(2)
   deg = rem(deg0,range(2));
elseif deg0 > range(1)
    deg = deg0;
else
    deg = range(2) - rem(abs(deg0),range(2));
end
   
end


