% �ж�����ǰ���ϵ
% ���룺����λ���ٶ�    �����׷������ǰ���1���ں����0
function direction = JudgePhase(chasePosVel,targetPosVel)
e = chasePosVel(1:3) - targetPosVel(1:3);
direction = sign(dot(e,chasePosVel(4:6)));   
end

