% 抵近预警
% 输入：coe_c - 我方星轨道根数                     coe_t - 目标星轨道根数    
%       warningDistance - 相对距离预警阈值         warningVelDistance - 速度方向距离预警阈值
%       startTime - 任务开始时间（高精度计算时给定）
% 输出：T1,T2 - 预警时间窗                          flag - 预警信号
function [T1,T2,flag,minDistance,minDistanceTime,minVelDistance,minVelDistanceTime] = dijingWarning(coe_c, coe_t, T, warningDistance, warningVelDistance, h, startTime)
global GM_Earth;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % 我方星初始位置速度     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % 目标星初始位置速度
model = [1 1 1];
smallRecursionNum = 30; % 小递推数量

[~,chaseData] = OrbitPrediction([r_c0;v_c0], T , smallRecursionNum * h, model,'RK7');    % 我方星速度位置数据表
[~,targetData] = OrbitPrediction([r_t0;v_t0], T , smallRecursionNum * h, model,'RK7');   % 目标星速度位置数据表

relativeData = targetData - chaseData;                                    % 每个时刻的相对位置速度
distanceData = sqrt(relativeData(1,:).^2 + relativeData(2,:).^2 + relativeData(3,:).^2);      % 两星距离
velDistanceData = zeros(1,length(chaseData)); % 初始化我方星速度方向上的距离

for i = 1 : length(chaseData)
    velDistanceData(i) = CalVelDistance(chaseData(:,i),targetData(1:3,i));          % 目标星在我方星速度方向上的距离
end  

T1=-1; T2 = inf; 
minDistance = inf;minDistanceTime = -1;
minVelDistance = inf;minVelDistanceTime = -1;

index1 = find(distanceData < warningDistance);                     % 距离阈值范围
index2 = find(velDistanceData < warningVelDistance);               % 速度方向上距离阈值范围

if ~(isempty(index1) && isempty(index2))
	%% 计算T1,T2
	flag = 1;
    if isempty(index1) % 只考虑速度方向上距离的预警
    index(1) = index2(1);
    index(2) = index2(end);
    elseif isempty(index2) % 只考虑距离预警
    index(1) = index1(1);
    index(2) = index1(end);
    else % 综合考虑两种因素预警
    index(1) = (index1(1) < index2(1)) * index1(1) + (index1(1) >= index2(1)) * index2(1);
    index(2) = (index1(end) > index2(end)) * index1(end) + (index1(end) <= index2(end)) * index2(end);
    end
	
	% 计算预警开始时刻
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

	% 计算预警结束时刻
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
	
	%% 计算最小距离及其时刻
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
	
	%% 计算速度方向最小距离及其时刻
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

    
% 小范围预警或无预警情况
else
	[~,minIndex3] = min(DistanceData);
	% 小范围预警
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
		
		if distance <= warningDistance || velDistance <= warningVelDistance % 计算T1
			T1 = smallRecursionNum * h * (minIndex3 - 1) + n * h;
			num = n;
			break;
		end
	end
	
	if T1 ~= -1 % 小范围预警发生在最小值的前一段
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
		
		if distance < minDistance % 最小距离时间
			minDistance = distance;
			minDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
		end
		
		if VelDistance < minVelDistance % 最小速度方向距离时间
			minVelDistance = VelDistance;
			minVelDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
		end
			
		if distance >= warningDistance && velDistance >= warningVelDistance % 计算T2
			T2 = smallRecursionNum * h * minIndex3 + n * h;
			break;
		end
	end
	
	else % 小范围预警发生在最小值的后一段或者无预警
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
		
		if T1 ~= -1 % 小范围预警发生在最小值的后一段
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
				
                if distance < minDistance % 最小距离时间
                    minDistance = distance;
                    minDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
                end
				
                if VelDistance < minVelDistance % 最小速度方向距离时间
                    minVelDistance = VelDistance;
                    minVelDistanceTime = smallRecursionNum * h * minIndex3 + n * h;
                end
				
                if distance >= warningDistance && velDistance >= warningVelDistance
					T2 = smallRecursionNum * h * minIndex3 + n * h;
					break;
                end
            end
			
		else % 无预警
			flag = 0;
			return;
		end
	end		
	flag = 0;
	return;
end
end

%% 距离在速度方向上的投影
% 输入：chasePosVel - 追踪星速度位置       targetPos - 目标星位置
function x = CalVelDistance(chasePosVel,targetPos)
i = chasePosVel(4:6) / norm(chasePosVel(4:6));                   % 追踪星速度方向
distance = chasePosVel(1:3) - targetPos;
x = norm(dot(distance,i));
end