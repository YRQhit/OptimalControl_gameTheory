% 点集中心
% 输入：data - 点集    
% 输出：P - 中心点
function [P,index] = FindCentralPoint(data)
[n,~] = size(data);
distanceData = pdist2(data,data);
temp = sum(distanceData,2)./n;
[~,index] = min(temp);  P = data(index,:);
end

