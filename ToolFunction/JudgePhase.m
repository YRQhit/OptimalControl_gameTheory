% 判断两星前后关系
% 输入：两星位置速度    输出：追踪星在前输出1、在后输出0
function direction = JudgePhase(chasePosVel,targetPosVel)
e = chasePosVel(1:3) - targetPosVel(1:3);
direction = sign(dot(e,chasePosVel(4:6)));   
end

