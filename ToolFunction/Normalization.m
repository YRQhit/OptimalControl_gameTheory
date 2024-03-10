% 归一化处理
% 输入: x - 原始数据(按列）   range - 归一化范围（默认0-1）
% 例：y = Normalization([1;2.2;3.5],[-1;1])
function y = Normalization(x,range)
[n, m] = size(x);   y = x;
Limit = zeros(2, m);
if nargin == 1
    range = Limit;
    range(1,:) = 0;   range(2,:) = 1;
end

for i = 1:m
    Limit(1,i) = min(x(:,i));       Limit(2,i) = max(x(:,i));
    k1 = Limit(2,i) - Limit(1,i);   k2 = range(2,i) - range(1,i);
    for j = 1:n
        y(j,i) = range(1,i) + k2 * (x(j,i) - Limit(1,i)) / k1;
    end
end
end

