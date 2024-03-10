% 计算太阳光照角（太阳指向目标星的矢量在目标星轨道平面的投影与追踪星到目标星的矢量夹角）
% 输入：x_c - 追踪星位置速度                    x_t - 目标星位置速度        
%      JD_startTime - 儒略日或时间行向量        second - 偏移时间（可以不给定）
% 输出：idealDegree - 理想太阳光照角（默认两星对地心夹角为0）     actualDegree - 实际太阳光照角
function [idealDegree,actualDegree] = IlluminationAngle(x_c,x_t,JD_startTime,second)
if length(JD_startTime) == 6
    JD = Mjday(JD_startTime(1),JD_startTime(2),JD_startTime(3),JD_startTime(4),JD_startTime(5),JD_startTime(6)) + 2400000.5;
else
    JD = JD_startTime;
end

if nargin == 4
    JD = JD + second / 86400;
end
t_norm_vector = cross(x_t(1:3) , x_t(4:6)) / norm(cross(x_t(1:3) , x_t(4:6))) ;     % 目标星轨道面单位法向量
sun_earth_vecotr =  -CalAstroBodyPos(JD);                                           % 太阳指向地球矢量
sun_target_vector = x_t(1:3) + sun_earth_vecotr;                                    % 太阳指向目标星矢量

% angleSym = dot(sun_target_vector , t_norm_vector);                          % 角度判别
% if angleSym >= 0
%    Sun_target_projection = sun_target_vector - angleSym / norm(t_norm_vector) * t_norm_vector;          %太阳矢量向目标星平面投影
% else
%    Sun_target_projection = sun_target_vector + angleSym / norm(t_norm_vector) * t_norm_vector;          %太阳矢量向目标星平面投影
% end
% idealDegree = acosd(dot(x_t(1:3) , Sun_target_projection) / (norm(x_t(1:3)) * norm(Sun_target_projection)));                % 太阳矢量投影与目标星位置矢量夹角(deg) - 理想光照角

idealDegree = acosd(dot(x_t(1:3) , sun_target_vector) / (norm(x_t(1:3)) * norm(sun_target_vector)));
chase_target_vector = x_t(1:3) - x_c(1:3);                                                              % 追踪星指向目标星矢量
% actualDegree = acosd(dot(chase_target_vector , Sun_target_projection) / (norm(chase_target_vector) * norm(Sun_target_projection)));   % 实际光照角
actualDegree = acosd(dot(chase_target_vector , sun_target_vector) / (norm(chase_target_vector) * norm(sun_target_vector)));  
end



