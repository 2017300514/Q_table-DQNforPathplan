
function reward = getReward2(action, pre_action, sensor, krasj,dist_a_obj,last_a_obj )
%function reward = getReward2(action, pre_action, sensor, krasj,dist_a_obj,last_a_obj ,dist_ave_limit)
%% 函数输入，输出说明
%{  
    输入：  action          智能体当前时刻的动作

            pre_action      智能体上一时刻的动作

            sensor          智能体当前时刻五个超声波传感器测量障碍物的距离 单位cm

            krasj           智能体状态信息，能够通过其中的值来判断智能体是否碰撞障碍物

            dist_a_o        智能体当前时刻距离目标点距离 单位米

            last_a_o        智能体上一时刻距离目标点距离 单位米

            dist_ave_limit  智能体奖励函数分段区间 （距离障碍物距离）单位米
    输出:
            reward          给予智能体奖励

    
%}

% 这里遗留一个 目标点转化后 的 上一时刻智能体到目标点距离重新定义问题
%  这里遗留一个 调参 参数放到函数外面问题
%  这里遗留一个重复计算左侧最近障碍物和右侧最近障碍物的问题 （待定，discretization可以用，但是DQN又没有这一过程）
%%

reward = 0;   %初始化奖励为 0

d_a_obsL = 100; %左侧障碍物最近距离 单位 厘米

d_a_obsR = 100; %右侧障碍物最近距离 单位 厘米  

status = ~isempty(find(krasj(2),1));   %判断智能体是否与环境障碍物边界碰撞  这里碰撞判定为 0.15

alpha = 1.5;

a = action;

pre_a = pre_action;

reward_crash = -2; 

x = min(sensor(1:3));

y = min(sensor(3:5));

temp = [d_a_obsL x];
d_a_obsL = temp((d_a_obsL > x) + 1);
temp = [d_a_obsR y];
d_a_obsR = temp((d_a_obsR > y) + 1);
temp = [d_a_obsL d_a_obsR];
if(~status )
    if (d_a_obsL < 100)
        if (d_a_obsR < 100)
            reward = exp(temp((d_a_obsL > d_a_obsR) + 1)/1000)*(last_a_obj-dist_a_obj) + ln(d_a_obsL/100) + ln(d_a_obsR/100);
        else 
            reward = exp(temp((d_a_obsL > d_a_obsR) + 1)/1000)*(last_a_obj-dist_a_obj) + ln(d_a_obsL/100);
        end
    else
        if (d_a_obsR < 100)
            reward = exp(temp((d_a_obsL > d_a_obsR) + 1)/1000)*(last_a_obj-dist_a_obj) + ln(d_a_obsR/100);
        else
            reward = alpha * (last_a_obj-dist_a_obj);
        end
    end
elseif(status)             %如果智能体碰撞障碍物，给予惩罚并跳出循环
    
    reward = reward_crash;
end

disp(reward);
end

