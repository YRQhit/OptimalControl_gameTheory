% ���������� - ͨ�������ٶȷ��������ά�����Ǿ��뱣��һ��
% ���룺rvc - ׷����λ���ٶ�      rvt - Ŀ����λ���ٶ�
% ������������
function [dv] = banfeicompensate(rvc,rvt)
global GM_Earth;
coe_c = State_rv_2_Orbit_Element(rvc(1:3),rvc(4:6),GM_Earth);
coe_t = State_rv_2_Orbit_Element(rvt(1:3),rvt(4:6),GM_Earth);
d0 = JudgePhase(rvc,rvt) * norm(rvc(1:3) - rvt(1:3));
T_1r = 2 * pi/ omegacal(norm(rvc(1:3)));

rvt_1r = OrbitPrediction(rvt,T_1r,60);
dv = 0;
if coe_c(1) > coe_t(1)
    dir  = -1;
else
    dir  = 1;
end
%% ����������ű�
e =[];
se = [];

ebest = 100;
% �ٶȸı��������½�
maxdv = 20;mindv = 0;
mark =1;            % �������
dvi_val = 0.0001;    % ���ε��ٶ���������λkm/s
for i = 1:50
    %% ���Ư��һ��ʱ��������λ���ٶȣ�B�Ǿ�������Ư�ɣ��ŵ�ѭ������
    rvc_1r = OrbitPrediction([rvc(1:3);(rvc(4:6) + dv * rvc(4:6)/norm(rvc(4:6)))],T_1r,60);
    
    %% ���Ƽ���Ư��һ��֮������Ǿ����
    val_d1 = norm(rvc_1r(1:3) - rvt_1r(1:3));
    d1 = JudgePhase(rvc_1r,rvt_1r) * val_d1;
    
    % ����
    e = [e;d1 - d0];
    se = [se;sign(d1 -d0)];
    if norm(e(i)) < ebest
        ebest = norm(e(i));
        optim.dv = dv;
        optim.e = e(i);
    end
    if norm(e(i)) < 5/1000
        break;
    end
    
    if abs(e(i)) / 10000 > dvi_val
        dv = dv + sign(e(i)) * dvi_val / 5;
    else
        dv = dv + 2e-5 * e(i);
    end
    
    % ��v һ����ʼ�仯
%     if i == 1
%         dv = sign(e(i)) * (norm(dv) + dvi_val);
%     end
%     if i ~= 1
%         if  sign(e(i)) * sign(e(1)) > 0     % ˵�� i������û�й����
%             % ��֤����Ǵӵ�һ����ʼ��������û�й�0�㣬�ۼ�ֱ�������
%             % ͬʱ������ۼӹ�������������������������ͨ��ǰ���break����
%             mindv = norm(dv);
% %           dv = dir * (norm(dv) + dvi_val/5);
%             if e(i) / 10000 > dvi_val
%                 dv = sign(e(i)) * (norm(dv) + dvi_val / 5);
%             else
%                 dv = sign(e(i)) * (norm(dv) + e(i) / 50000);
%             end
%         end
% 
%         if sign(e(i)) * sign(e(1)) < 0
%             %% ������dvС�� maxdv����maxdv �������ڵ�����ʱ����ֵ��mindv ��maxdv֮��
%             if norm(dv) < maxdv
%                 maxdv = norm(dv);
%                 ddv = (maxdv-mindv) / 10;
%             end
%             dv = sign(e(i)) * (norm(dv) - norm(ddv));
%         end
%     end
end
dv = optim.dv;
end