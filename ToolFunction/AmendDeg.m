% �Ƕ��������������ݲ�ͬҪ��ĽǶȷ�Χ����ʵ�ʽǶ�ֵ
% ���룺deg0 - ԭʼ�Ƕ�      type - �Ƕȷ�Χ
% �����deg - ���㷶ΧҪ��ĽǶ�ֵ
% ���� - STK����ؽǶȵķ�Χ��
%     �����ǣ�"0 - 180"            ���ص���ǣ�"0 - 360"      
%     ������ྭ��"-180 - 360"       �����ǣ�"-180 - 360" 
function deg = AmendDeg(deg0, type)
if nargin == 1
    type = "-180 - 360";
end

if strcmp(type,"0 - 180")
    range = [0 180];
elseif strcmp(type,"0 - 360")
    range = [0 360];
elseif strcmp(type,"-180 - 360")
    range = [-180 360];
else
    disp("�÷�Χ����");
    return;
end    

if deg0 > range(2)
   deg = rem(deg0,range(2));
elseif deg0 > range(1)
    deg = deg0;
else
    deg = range(2) - rem(abs(deg0),range(2));
end
   
end


