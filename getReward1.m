
function [reward,arrive_check] = getReward1(action, pre_rot_punkt, rot_punkt,goal ,sensor, krasj,dist_a_obj ,last_a_obj)
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

            arrive_check    判断智能体是否到达目标点
    
%}
%%
reward = 0;   %初始化奖励为 0

arrive_check = 0; % 初始化为没有到达目标点 = 0

status = ~isempty(find(krasj(2),1));   %判断智能体是否与环境障碍物边界碰撞  这里碰撞判定为 0.15

% alpha = 1;
dist_x = abs(pre_rot_punkt(1)-goal(1))-abs(rot_punkt(1)-goal(1));

dist_y = abs(pre_rot_punkt(2)-goal(2))-abs(rot_punkt(2)-goal(2));

a = action;

reward_crash = -100;

if(~status )

    if dist_x > 0
        if dist_y >0 
            dist_reward = (last_a_obj - dist_a_obj) * 0.2;
        else
            dist_reward = dist_x * 0.05;
        end
    elseif dist_y >0
            dist_reward = dist_y * 0.05 ;
    else
        dist_reward =  (last_a_obj - dist_a_obj) * 0.2;
    end
    reward = roundn(reward + dist_reward*50,-4);
    if dist_a_obj < 2
        reward = 100;
        arrive_check = 1;
    end
%     
else             %如果智能体碰撞障碍物，给予惩罚并跳出循环
        
    reward = reward_crash;
    
    
   
end
disp(reward); 
end   
