% ����̫�����սǣ�̫��ָ��Ŀ���ǵ�ʸ����Ŀ���ǹ��ƽ���ͶӰ��׷���ǵ�Ŀ���ǵ�ʸ���нǣ�
% ���룺x_c - ׷����λ���ٶ�                    x_t - Ŀ����λ���ٶ�        
%      JD_startTime - �����ջ�ʱ��������        second - ƫ��ʱ�䣨���Բ�������
% �����idealDegree - ����̫�����սǣ�Ĭ�����ǶԵ��ļн�Ϊ0��     actualDegree - ʵ��̫�����ս�
function [idealDegree,actualDegree] = IlluminationAngle(x_c,x_t,JD_startTime,second)
if length(JD_startTime) == 6
    JD = Mjday(JD_startTime(1),JD_startTime(2),JD_startTime(3),JD_startTime(4),JD_startTime(5),JD_startTime(6)) + 2400000.5;
else
    JD = JD_startTime;
end

if nargin == 4
    JD = JD + second / 86400;
end
t_norm_vector = cross(x_t(1:3) , x_t(4:6)) / norm(cross(x_t(1:3) , x_t(4:6))) ;     % Ŀ���ǹ���浥λ������
sun_earth_vecotr =  -CalAstroBodyPos(JD);                                           % ̫��ָ�����ʸ��
sun_target_vector = x_t(1:3) + sun_earth_vecotr;                                    % ̫��ָ��Ŀ����ʸ��

% angleSym = dot(sun_target_vector , t_norm_vector);                          % �Ƕ��б�
% if angleSym >= 0
%    Sun_target_projection = sun_target_vector - angleSym / norm(t_norm_vector) * t_norm_vector;          %̫��ʸ����Ŀ����ƽ��ͶӰ
% else
%    Sun_target_projection = sun_target_vector + angleSym / norm(t_norm_vector) * t_norm_vector;          %̫��ʸ����Ŀ����ƽ��ͶӰ
% end
% idealDegree = acosd(dot(x_t(1:3) , Sun_target_projection) / (norm(x_t(1:3)) * norm(Sun_target_projection)));                % ̫��ʸ��ͶӰ��Ŀ����λ��ʸ���н�(deg) - ������ս�

idealDegree = acosd(dot(x_t(1:3) , sun_target_vector) / (norm(x_t(1:3)) * norm(sun_target_vector)));
chase_target_vector = x_t(1:3) - x_c(1:3);                                                              % ׷����ָ��Ŀ����ʸ��
% actualDegree = acosd(dot(chase_target_vector , Sun_target_projection) / (norm(chase_target_vector) * norm(Sun_target_projection)));   % ʵ�ʹ��ս�
actualDegree = acosd(dot(chase_target_vector , sun_target_vector) / (norm(chase_target_vector) * norm(sun_target_vector)));  
end



