function newTime = UpdateTime(currentTime, years, months, days, hours, minutes, seconds)
    % ����ʱ��
    % ���룺
    %   - currentTime: ��ǰʱ�䣬��ʽΪ [��, ��, ��, ʱ, ��, ��]
    %   - years: Ҫ���ӻ���ٵ�����
    %   - months: Ҫ���ӻ���ٵ�����
    %   - days: Ҫ���ӻ���ٵ�����
    %   - hours: Ҫ���ӻ���ٵ�Сʱ��
    %   - minutes: Ҫ���ӻ���ٵķ�����
    %   - seconds: Ҫ���ӻ���ٵ�����
    % �����
    %   - newTime: ���º��ʱ�䣬��ʽΪ [��, ��, ��, ʱ, ��, ��]
    
    % ����ǰʱ��ת��Ϊ MATLAB ���õ�ʱ���ʽ
    currentDatetime = datetime(currentTime);
    
    % ʹ�� datetime ��������ʱ����������
    newDatetime = currentDatetime + years*365*24*60*60 + months*30*24*60*60 + days*24*60*60 + hours*60*60 + minutes*60 + seconds;
    
    % ���µ�ʱ��ת��Ϊ�Զ������ʽ [��, ��, ��, ʱ, ��, ��]
    newTime = [year(newDatetime), month(newDatetime), day(newDatetime), hour(newDatetime), minute(newDatetime), second(newDatetime)];
end


%ʾ��
% currentTime = [2022, 1, 1, 0, 0, 0];  % ��ǰʱ�� [��, ��, ��, ʱ, ��, ��]
% years = 1;  % ���ӻ���ٵ�����
% months = 0;  % ���ӻ���ٵ�����
% days = 0;  % ���ӻ���ٵ�����
% hours = 0;  % ���ӻ���ٵ�Сʱ��
% minutes = 0;  % ���ӻ���ٵķ�����
% seconds = 0;  % ���ӻ���ٵ�����
% 
% newTime = UpdateTime(currentTime, years, months, days, hours, minutes, seconds);
% 
% disp(newTime);
