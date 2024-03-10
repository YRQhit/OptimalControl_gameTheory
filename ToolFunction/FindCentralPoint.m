% �㼯����
% ���룺data - �㼯    
% �����P - ���ĵ�
function [P,index] = FindCentralPoint(data)
[n,~] = size(data);
distanceData = pdist2(data,data);
temp = sum(distanceData,2)./n;
[~,index] = min(temp);  P = data(index,:);
end

