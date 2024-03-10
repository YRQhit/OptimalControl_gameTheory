function newTime = UpdateTime(currentTime, years, months, days, hours, minutes, seconds)
    % 更新时间
    % 输入：
    %   - currentTime: 当前时间，形式为 [年, 月, 日, 时, 分, 秒]
    %   - years: 要增加或减少的年数
    %   - months: 要增加或减少的月数
    %   - days: 要增加或减少的天数
    %   - hours: 要增加或减少的小时数
    %   - minutes: 要增加或减少的分钟数
    %   - seconds: 要增加或减少的秒数
    % 输出：
    %   - newTime: 更新后的时间，形式为 [年, 月, 日, 时, 分, 秒]
    
    % 将当前时间转换为 MATLAB 内置的时间格式
    currentDatetime = datetime(currentTime);
    
    % 使用 datetime 函数进行时间增减运算
    newDatetime = currentDatetime + years*365*24*60*60 + months*30*24*60*60 + days*24*60*60 + hours*60*60 + minutes*60 + seconds;
    
    % 将新的时间转换为自定义的形式 [年, 月, 日, 时, 分, 秒]
    newTime = [year(newDatetime), month(newDatetime), day(newDatetime), hour(newDatetime), minute(newDatetime), second(newDatetime)];
end


%示例
% currentTime = [2022, 1, 1, 0, 0, 0];  % 当前时间 [年, 月, 日, 时, 分, 秒]
% years = 1;  % 增加或减少的年数
% months = 0;  % 增加或减少的月数
% days = 0;  % 增加或减少的天数
% hours = 0;  % 增加或减少的小时数
% minutes = 0;  % 增加或减少的分钟数
% seconds = 0;  % 增加或减少的秒数
% 
% newTime = UpdateTime(currentTime, years, months, days, hours, minutes, seconds);
% 
% disp(newTime);
