% �ֽ�Ԥ��
% ���룺coe_c - �ҷ��ǹ������                     coe_t - Ŀ���ǹ������    
%       warningDistance - ��Ծ���Ԥ����ֵ         warningVelDistance - �ٶȷ������Ԥ����ֵ
%       startTime - ����ʼʱ�䣨�߾��ȼ���ʱ������
% �����T1,T2 - Ԥ��ʱ�䴰                          flag - Ԥ���ź�
function [T1,T2,flag,minDistance,minDistanceTime,minVelDistance,minVelDistanceTime] = dijingWarning(coe_c, coe_t, T, warningDistance, warningVelDistance, h, startTime)
global GM_Earth;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % �ҷ��ǳ�ʼλ���ٶ�     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % Ŀ���ǳ�ʼλ���ٶ�
model = [1 1 1];
smallRecursionNum = 30; % С��������

[~,chaseData] = OrbitPrediction([r_c0;v_c0], T , smallRecursionNum * h, model,'RK7');    % �ҷ����ٶ�λ�����ݱ�
[~,targetData] = OrbitPrediction([r_t0;v_t0], T , smallRecursionNum * h, model,'RK7');   % Ŀ�����ٶ�λ�����ݱ�

relativeData = targetData - chaseData;                                    % ÿ��ʱ�̵����λ���ٶ�
distanceData = sqrt(relativeData(1,:).^2 + relativeData(2,:).^2 + relativeData(3,:).^2);      % ���Ǿ���
velDistanceData = zeros(1,length(chaseData)); % ��ʼ���ҷ����ٶȷ����ϵľ���

for i = 1 : length(chaseData)
    velDistanceData(i) = CalVelDistance(chaseData(:,i),targetData(1:3,i));          % Ŀ�������ҷ����ٶȷ����ϵľ���
end  

T1=-1; T2 = inf; 
minDistance = inf;minDistanceTime = -1;
minVelDistance = inf;minVelDistanceTime = -1;

index1 = find(distanceData < warningDistance);                     % ������ֵ��Χ
index2 = find(velDistanceData < warningVelDistance);               % �ٶȷ����Ͼ�����ֵ��Χ

if ~(isempty(index1) && isempty(index2))
	%% ����T1,T2
	flag = 1;
    if isempty(index1) % ֻ�����ٶȷ����Ͼ����Ԥ��
    index(1) = index2(1);
    index(2) = index2(end);
    elseif isempty(index2) % ֻ���Ǿ���Ԥ��
    index(1) = index1(1);
    index(2) = index1(end);
    else % �ۺϿ�����������Ԥ��
    index(1) = (index1(1) < index2(1)) * index1(1) + (index1(1) >= index2(1)) * index2(1);
    index(2) = (index1(end) > index2(end)) * index1(end) + (index1(end) <= index2(end)) * index2(end);
    end
	
	% ����Ԥ����ʼʱ��
    %testTime2 = AddTime(startTime , index(1) * smallRecursionNum * h);
    %testTime3 = AddTime(startTime , index(2) * smallRecursionNum * h);
	if index(1) ~= 1
		chasePosVel = chaseData(:,index(1) - 1);
		targetPosVel = targetData(:,index(1) - 1);
		for n = 1:smallRecursionNum
			if nargin == 6
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
			elseif nargin == 7
				startTimeNew1 = AddTime(startTime , (n - 1)  * h);
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew1);
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew1);
			end
			distance = norm(targetPosVel - chasePosVel);
			velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
			if distance <= warningDistance || velDistance <= warningVelDistance
				T1 = smallRecursionNum * h * (index(1) - 1) + n * h;
				break;
			end
		end
	else
		T1 = 0;
	end

	% ����Ԥ������ʱ��
    if index(2) ~= length(chaseData)
		chasePosVel = chaseData(:,index(2));
		targetPosVel = targetData(:,index(2));
		for n = 1:smallRecursionNum
			if nargin == 6
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
			elseif nargin == 7
				startTimeNew2 = AddTime(startTime , (n - 1)  * h);
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew2);
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew2);
			end
			distance = norm(targetPosVel - chasePosVel);
			velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
			if distance >= warningDistance && velDistance >= warningVelDistance
				T2 = smallRecursionNum * h * index(end) + n * h;
				break;
			end
		end
	else
		T2 = T; 
    end
	
	%% ������С���뼰��ʱ��
	[~,minIndex1] = min(distanceData);
    % testTime1 = AddTime(startTime , smallRecursionNum * h * (minIndex1-1));
	chasePosVel = chaseData(:,minIndex1 - 1);
	targetPosVel = targetData(:,minIndex1 - 1);
	for n = 1:2 * smallRecursionNum
		if nargin == 6
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
		elseif nargin == 7
			startTimeNew21 = AddTime(startTime , (n - 1)  * h);
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew21);
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew21);
		end
		distance = norm(targetPosVel - chasePosVel);
		
        if distance < minDistance
            minDistance = distance;
            minDistanceTime = smallRecursionNum * h * (minIndex1 - 1) + n * h;
        end
	end
	
	%% �����ٶȷ�����С���뼰��ʱ��
	[~,minIndex2] = min(velDistanceData);
	chasePosVel = chaseData(:,minIndex2 - 1);
	targetPosVel = targetData(:,minIndex2 - 1);
    for n = 1:2 * smallRecursionNum
		if nargin == 6
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
		elseif nargin == 7
			startTimeNew22 = AddTime(startTime , (n - 1)  * h);
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew22);
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew22);
		end
		velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
		
        if velDistance < minVelDistance
            minVelDistance = velDistance;
            minVelDistanceTime = smallRecursionNum * h * (minIndex2 - 1) + n * h;
        end
    end

    
% С��ΧԤ������Ԥ�����
else
	[~,minIndex3] = min(DistanceData);
	% С��ΧԤ��
	chasePosVel = chaseData(:,minIndex3 - 1);
	targetPosVel = targetData(:,minIndex3 - 1);
	for n = 1:smallRecursionNum
		if nargin == 6
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
		elseif nargin == 7
			startTimeNew3 = AddTime(startTime , (n - 1)  * h);
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew3);
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew3);
		end
		distance = norm(targetPosVel - chasePosVel);
		velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
		
		if distance <= warningDistance || velDistance <= warningVelDistance % ����T1
			T1 = smallRecursionNum * h * (minIndex3 - 1) + n * h;
			num = n;
			break;
		end
	end
	
	if T1 ~= -1 % С��ΧԤ����������Сֵ��ǰһ��
		for n = num:smallRecursionNum
		if nargin == 6
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
		elseif nargin == 7
			startTimeNew4 = AddTime(startTime , (n - 1)  * h);
			chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew4);
			targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew4);
		end
		distance = norm(targetPosVel - chasePosVel);
		velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
		
		if distance < minDistance % ��С����ʱ��
			minDistance = distance;
			minDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
		end
		
		if VelDistance < minVelDistance % ��С�ٶȷ������ʱ��
			minVelDistance = VelDistance;
			minVelDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
		end
			
		if distance >= warningDistance && velDistance >= warningVelDistance % ����T2
			T2 = smallRecursionNum * h * minIndex3 + n * h;
			break;
		end
	end
	
	else % С��ΧԤ����������Сֵ�ĺ�һ�λ�����Ԥ��
		chasePosVel = chaseData(:,minIndex3);
		targetPosVel = targetData(:,minIndex3);
		for n = 1:smallRecursionNum
			if nargin == 6
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
			elseif nargin == 7
				startTimeNew5 = AddTime(startTime , (n - 1)  * h);
				chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew5);
				targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew5);
			end
			distance = norm(targetPosVel - chasePosVel);
			velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
			if distance <= warningDistance || velDistance <= warningVelDistance
				T1 = smallRecursionNum * h * minIndex3 + n * h;
				num = n;
				break;
			end
		end
		
		if T1 ~= -1 % С��ΧԤ����������Сֵ�ĺ�һ��
            for n = num:smallRecursionNum
				if nargin == 6
					chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7'); 
					targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7');
				elseif nargin == 7
					startTimeNew6 = AddTime(startTime , (n - 1)  * h);
					chasePosVel = OrbitPrediction(chasePosVel, h , h, model,'RK7',startTimeNew6);
					targetPosVel = OrbitPrediction(targetPosVel, h , h, model,'RK7',startTimeNew6);
				end
				distance = norm(targetPosVel - chasePosVel);
				velDistance = CalVelDistance(chasePosVel, targetPosVel(1:3));
				
                if distance < minDistance % ��С����ʱ��
                    minDistance = distance;
                    minDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
                end
				
                if VelDistance < minVelDistance % ��С�ٶȷ������ʱ��
                    minVelDistance = VelDistance;
                    minVelDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
                end
				
                if distance >= warningDistance && velDistance >= warningVelDistance
					T2 = smallRecursionNum * h * minIndex3 + n * h;
					break;
                end
            end
			
		else % ��Ԥ��
			flag = 0;
			return;
		end
	end		
	flag = 0;
	return;
end
end

%% �������ٶȷ����ϵ�ͶӰ
% ���룺chasePosVel - ׷�����ٶ�λ��       targetPos - Ŀ����λ��
function x = CalVelDistance(chasePosVel,targetPos)
i = chasePosVel(4:6) / norm(chasePosVel(4:6));                   % ׷�����ٶȷ���
distance = chasePosVel(1:3) - targetPos;
x = norm(dot(distance,i));
end