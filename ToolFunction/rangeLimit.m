% �������ͺ��� 
% ���룺x - ������   range - ��Χ
% ����y = rangeLimit([-0.1 2 5 -2.5],[0 0 0 0;1 1 1 1])
function y = rangeLimit(x,range)
n = length(x);  y = x;
for i = 1:n
    if x(i) < range(1,i)
        y(i) = range(1,i);
    elseif x(i) > range(2,i)
        y(i) = range(2,i);
    end
end
end

