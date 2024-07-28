
function [reward, goal,decide_2,decide_3] = nnGetReward1_forquestion(action, prev_action, sensor2, sensor1, krasj,circle_plot1,circle_plot2,decide_2,decide_3)
%action 是小车当前时刻的动作
%pre_action是小车上一时刻的动作

%sensor1是小车当前时刻五个超声波传感器测量障碍物的距离
%sensor2是小车上一时刻五个超声波传感器测量障碍物的距离
% krasj 记录小车状态信息，能够通过其中的值来判断小车是否碰撞障碍物
% circle_plot1  小车当前时刻的位置
% circle_plot2  小车上一时刻的位置
% decide_2  判断小车是否在目标点 45dm内
% decide_3  判断小车是否在目标点22dm内

a = action;   %记录小车的动作，直行取1，左转取2，右转取3
reward = 0;   %初始化奖励为0
goal = false; %初始化默认小车没有到达目标点
final_goal=[75,30];     %设定目标点坐标

dist=norm(final_goal-circle_plot1);    %计算当前时刻小车与目标点距离
dist1=norm(final_goal-circle_plot2);   %记录上一时刻小车与目标点距离
status = ~isempty(find(krasj(2),1));   %判断小车是否与环境障碍物边界碰撞

r = [0.01, -0.01, -0.9, 0.9];   %奖励向量，存放不同的奖励值

switch(a)                       %判断动作，这里鼓励直行，避免一直左转或一直右转
    case(1)
        reward=reward+r(1);
    case(2)
        reward=reward+r(2);
    case(3)
        reward=reward+r(2);
end

if(decide_2==0 && dist<=45)      %这里鼓励小车向目标点靠近
    decide_2 = 1;
    reward = reward+0.5*r(4);
end
if(decide_2==1 && dist>45)
    decide_2 = 0;
    reward= reward- 0.5*r(4);
end

if(decide_3==0 && dist<=22)         %这里鼓励小车继续向目标点靠近
    decide_3 = 1;
    reward = reward+0.75*r(4);
end
if(decide_3==1 && dist>=22)
    decide_3 = 0;
    reward = reward-0.75*r(4);
end

if(decide_2==0)                %这里为了避免小车一直在远离目标点的区域探索
    reward=reward+r(2);
end
if(decide_3==0)                 %这里为了避免小车一直在远离目标点的区域探索
    reward=reward+r(2); 
end
if(decide_3==0 && dist1 > dist)
    reward=reward+r(1);
end
if(prev_action == 2 && a == 3)   %这里避免小车左右摇晃，左转后立刻右转
    
    reward = reward + 4*r(2);
elseif(prev_action == 3 && a == 2)  %这里避免小车左右摇晃，右转后立刻左转
    
    reward = reward + 4*r(2);

end


if(status)             %如果小车碰撞障碍物，给予惩罚并跳出循环
    
    reward = r(3);
end

if (sum(sensor1 - sensor2) >= 0)   %这里奖励小车避开障碍物，使得超声波传感器内障碍物距离变小
    
    reward = reward+ 3*r(1);
else
    
    reward = reward+ 3*r(2);
end
if(dist <= 2)           %这里是判定小车到达目标点附近，给予奖励并跳出循环
    goal = true;
    disp(1);
    reward = reward+r(4);
end

end