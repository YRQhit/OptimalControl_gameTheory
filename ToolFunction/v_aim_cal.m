%���Բ������ٶ�
%���룺x_aim - λ��ʸ��   h - �����λ�Ƕ���
function [v_aim]=v_aim_cal(x_aim,h)
global GM_Earth;
val_v=sqrt(GM_Earth/norm(x_aim));
dir_v_aim=cross(h,x_aim)/norm(cross(h,x_aim));          %v_aim����ĵ�λʸ��

v_aim(1)=dir_v_aim(1)*val_v;
v_aim(2)=dir_v_aim(2)*val_v;
v_aim(3)=dir_v_aim(3)*val_v;

end