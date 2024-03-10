% ������תƽ�����
% ���룺e - ƫ����  trueAno - �����ǣ�deg)
% �����Me - ƽ����ǣ�deg)
function Me = true2Mean(e, trueAno)
global rad2deg deg2rad;
trueAno = trueAno * deg2rad;
E = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(trueAno / 2)); 
Me = (E - e * sin(E)) * rad2deg;
end

