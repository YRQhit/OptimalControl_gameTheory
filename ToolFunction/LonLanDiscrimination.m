% �ж�ĳ���Ƿ�λ���й�����
% ���룺P = [���ȣ�γ��]
% �����InChina - true/false
function InChina = LonLanDiscrimination(P, position, R)
global lon_lan
InChina = 0;
lon = P(1);lan = P(2);

if nargin == 1
    if lon < 74 || lon > 134
        return;
    end
    if lan < 15 || lan > 53
        return;
    end
    lonlist = lon_lan(:,1);
    [index,~] = find(lonlist > lon-2 & lonlist < lon + 2);      % �ڸù������5�ȵľ�γ����  
    n = length(index);
    for i = 1:n
        if (lon_lan(index(i),2) >lan - 1 && lon_lan(index(i),2) < lan + 1)
            InChina = 1;
            break;
        end
    end
end

if nargin == 3
   if  Haversine(P, position) < R
       InChina = 1;
   end
end
end
%% �������㾭γ�������
function distance = Haversine(P1, P2)
global deg2rad r_E;
lon1 = P1(1) * deg2rad;    lon2 = P2(1) * deg2rad;
lan1 = P1(2) * deg2rad;    lan2 = P2(2) * deg2rad;

haversin = @(x)(sin(x / 2)^2);
h = haversin(lan2 - lan1) + cos(lan1) * cos(lan2) * haversin(lon1 - lon2);
distance = 2 * r_E * asin(sqrt(h));
end










