function newTime = AddTime(Time , addsecond)
logic = sign(addsecond);                                           % ��־ʱ��ǰ��/����   
addsecond = abs(addsecond);           addSecond = round(addsecond);   
addMinute = floor(addSecond / 60);    addSecond = addSecond - addMinute * 60;
addHour = floor(addMinute / 60);      addMinute = addMinute - addHour * 60;   
addDay = floor(addHour / 24);         addHour = addHour - addDay * 24;
newTime = Time + logic * [0 0 addDay addHour addMinute addSecond];
newTime = AmendTime(newTime);
end
%% ����ʱ���ʽ
function newTime = AmendTime(startTime)
dayOfMonth = [31 31 28 31 30 31 30 31 31 30 31 30 31];         % ����������12�µ�12�£�
if(rem(startTime(2),4) == 0)
    dayOfMonth(3) = dayOfMonth(2) + 1;
end

if startTime(6) >= 60                                       % ������           
    startTime(6) = startTime(6) - 60;
    startTime(5) = startTime(5) + 1;
elseif startTime(6) < 0 
    startTime(6) = 60 + startTime(6);
    startTime(5) = startTime(5) - 1;
end

if startTime(5) >= 60                                       % ������
    startTime(5) = startTime(5) - 60;
    startTime(4) = startTime(4) + 1;
elseif startTime(5) < 0 
    startTime(5) = 60 + startTime(5);
    startTime(4) = startTime(4) - 1;
end

if startTime(4) >= 24                                       % ����ʱ
   startTime(4) = startTime(4) - 24;
   startTime(3) = startTime(3) + 1;
elseif startTime(4) < 0
    startTime(4) = 24 + startTime(4);
    startTime(3) = startTime(3) - 1;
end

if startTime(3) > dayOfMonth(startTime(2) + 1)                  % ������
   startTime(3) = startTime(3) - dayOfMonth(startTime(2) + 1);
   startTime(2) = startTime(2) + 1;
elseif startTime(3) <= 0
    startTime(3) = dayOfMonth(startTime(2)) + startTime(3);
    startTime(2) = startTime(2) - 1;
end

if startTime(2) >= 13                                         % ������
   startTime(2) = startTime(2) - 12;
   startTime(1) = startTime(1) + 1;
elseif startTime(2) <= 0
    startTime(2) = 12 + startTime(2);
    startTime(1) = startTime(1) - 1;
end
newTime = startTime;
end
