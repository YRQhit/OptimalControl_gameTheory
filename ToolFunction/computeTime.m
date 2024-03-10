% 计算各段时间
% 输入：T - 分段点      Total - 总时间
% 例：t = computeTime([0.36 0.72],1000);
function t = computeTime(T,Total)
n = length(T);
t = zeros(1,n+1);
for i = 1:n+1
    if i == 1
        t(1) = T(1) * Total;
    elseif i == n+1
        t(n+1) = (1 - T(n))*Total;
    else
        t(i) = (T(i) - T(i-1))*Total;
    end
end
end

