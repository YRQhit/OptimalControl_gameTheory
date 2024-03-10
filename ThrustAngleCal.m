%���룺����ϵ���ٶ�����   ��ǰRV
%���������Ƕ�alpha--��xoy�н� 0--90 �� beta--xoyͶӰ��x��н�  0--360  Thrust_Angle = [alpha; beta]  deg      

function Thrust_Angle = ThrustAngleCal(RV, deltv)  
    global rad2deg
    RotMat1 = J20002VVLH(RV(1:3), RV(4:6));  % ���ж�Ӧ���ϵxyz��������
    
    Rx = dot(deltv, RotMat1(1, :)) * RotMat1(1, :);
    Ry = dot(deltv, RotMat1(2, :)) * RotMat1(2, :);
    
    beta = atan(Ry / Rx) * rad2deg;
    
    if(dot(RotMat1 * deltv, RotMat1(1,:)) < 0 && dot(RotMat1 * deltv, RotMat1(2,:)) > 0) 
        beta = 180 + beta;
    elseif (dot(RotMat1 * deltv, RotMat1(1,:)) < 0 && dot(RotMat1 * deltv, RotMat1(2,:)) < 0) 
        beta = - beta - 180;
    elseif (dot(RotMat1 * deltv, RotMat1(1,:)) > 0 && dot(RotMat1 * deltv, RotMat1(2,:)) < 0) 
        beta = beta + 360;               
    end
    alpha = 90 - acos(dot(RotMat1 * deltv, RotMat1(3,:)))/(norm(RotMat1 * deltv) * norm(RotMat1(3,:))) * rad2deg;
    Thrust_Angle = [alpha; beta];
end