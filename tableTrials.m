function [Q, tot_reward, i, getgoal,n_crash,n_arrive, info_s2,figure_z,pool,pool_num] = tableTrials(Q, alpha, gamma, T,stadium_option, n_arrive, tot_rot ,h_car,  ...
                                                                              h_circ, h_poly,state_space, action_list,max_steps,   ...
                                                                              n_crash,info_s1, h_text,init_place,goal_list,goal_num,figure_z,pool,pool_num)

global mode;
global figure1;

%% ----- Initialize parameters

v_car =  0.3;      % 小车运行速度 max 直行速度

vector = 0.1;      % 动态障碍物移动速度

pool_max =  10000;

sensor_lengde = 10;  

% 不需要调整的参数，只是为了程序初始化

getgoal = 0;       % 用来判定是否到达目标点（引导点）以及根据值判断到达第几个目标点


arrive_check = 0;

tot_reward = 0;    % 总奖励

goal = goal_list(1,:);           % 智能体第一个目标位置

path_x =  zeros(1,1000);         % 初始化轨迹x坐标 向量
path_y =  zeros(1,1000);         % 初始化轨迹y坐标 向量
path_x(1) = init_place(1);       % 记录路径坐标的x
path_y(1) = init_place(2);       % 记录路径坐标的y

rot_punkt= init_place;       % 智能体位置

ang = table_cos_to_angle( h_car,rot_punkt,goal);  % 初始化方向角

ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);

Q_temp = Q;                  % Q-table  迭代前的Q-table

sensor = checkCrash(h_car, h_poly, h_circ);         % 初始记录传感器的值

state = discretization(state_space, sensor(1:5), stadium_option, ang);     % 初始化离散状态值

dist_a_obj = norm(goal - rot_punkt); 

pool_copy = pool;

poolNum_copy = pool_num;

max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % 从起始点到达目标点的最多步数

step_goal = 1;
%% ----- Trial with max_steps
    for i = 1:max_steps
%% 暂停仿真模块     
    if( strcmp(mode,'Pause1') )
%             i = 0;
        tot_reward = 0;
        Q = Q_temp;
        break;
    end

%% 动态障碍物运动模块
    if( strcmp(stadium_option, 'Dynamic') == 1 )
        rot1 = atan(0.5);
        rot2 = -rot1;
        if(mod(i,540)>270)
            rot1 = atan(0.5)+pi;
            rot2 = rot2+pi;
        end
        moveworker(h_circ(4),rot1,vector);
        moveworker(h_circ(5),rot2,vector);
    end
%% 记录上一时刻状态输入的数据
        last_a_obj = dist_a_obj;
        pre_rot_punkt = rot_punkt;           % 上一时刻位置
        temp_sensor = sensor;
        pre_ang_dqn = ang_dqn;
%%  Q-table 产生Q值，进行决策
       
        % a = tableEpsilonGreedyExploration(Q, state, action_list, epsilon);
        a = softMaxSelection(Q, state, action_list, T);

        step_goal = step_goal + 1;
        
        [tot_rot,rot_punkt]= doActionCircle(a, tot_rot, v_car,  h_car);
                                                                                    % [guide_goal,local_pos]=getCarposition(rot_punkt);
        dist_a_obj = norm(goal - rot_punkt);
        
        [sensor, crash] = checkCrash(h_car, h_poly, h_circ);                % 判断碰撞和获取传感器数值

        ang = table_cos_to_angle( h_car,rot_punkt,goal);                     %计算视线角
        
        [reward ,arrive_check]= getReward1(a,pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
        
        % 奖励函       数得改
        [next_state, k1, k2, k3, k4] = discretization(state_space, sensor(1:5), stadium_option, ang);

        % 更新Q表
        Q = updateQTable(state, a, reward, Q, next_state, alpha, gamma, arrive_check);
        
        state = next_state;
        % 用来放入池子的数据
        ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);
        
        [reward_dqn,~] = nnGetReward1(a, pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
%% 记录数据，放入DQN池子
        pool_count = poolNum_copy;                % 定义新数据放入池子的位置

        if poolNum_copy > pool_max                    % 如果进来的数据超过池子容量，有效数据就是池子大小           
           pool_count =  rem(pool_count,pool_max)+(rem(pool_count,pool_max)==0)*pool_max;
        end
        
        pool_copy(:,pool_count) = [sensor,ang_dqn,dist_a_obj,temp_sensor,pre_ang_dqn,last_a_obj,reward_dqn,arrive_check,a];
        
        poolNum_copy = poolNum_copy+1;
%% 输出信息整理
        path_x(i+1) = rot_punkt(1);
        path_y(i+1) = rot_punkt(2);
        if( ~isempty(find(crash(2),1)) ); n_crash = n_crash + 1; end
        tot_reward = tot_reward + reward;
        switch(a)
            case 1, action = 'forward';
            case 2, action = 'right';
            case 3, action = 'left';
        end
        info_s2 = sprintf('%s%10d%24s%8d%18s%17s', 'l_zone:', k1, 'r_zone:', k3, 'Action:', action);
        info_s2 = sprintf('%s\n%s%10d%24s%8d%14s%10d', info_s2, 'l_sector:', k2, 'r_sector:', k4, 'Crash:', n_crash);
        info_s  = sprintf('%s\n%s', info_s1, info_s2);
        set(h_text, 'String', info_s);
        
%% 对于到达目标点后的处理  
    if dist_a_obj < 2 
        n_arrive = n_arrive+1;                      % 记录到达目标点次数
        getgoal = getgoal + 1 ;                     % 到达之后更新到达目标序号
        pool = pool_copy;
        pool_num = poolNum_copy;
        if getgoal < goal_num
            goal = goal_list(getgoal+1,:);            %  更新目标 跟目标有关的量都要更新
                       
            dist_a_obj = norm(goal-rot_punkt);      % 重新计算上一时刻距离目标点距离
            
            ang = table_cos_to_angle( h_car,rot_punkt,goal);   %视线角也重新设置
            
            state = discretization(state_space, sensor, stadium_option, ang);
            
            %用来放入池子的数据也要更新
            ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);
           
            max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % 从起始点到达目标点的最多步数
            
            step_goal = 1;
        else
            figure_z = figure_z + 1;
            drawPath(figure_z, path_x(1:i+1), path_y(1:i+1),stadium_option);
            hold all
            plot(goal_list(:,1),goal_list(:,2),'d','color',[0.93 0.69 0.13],'markersize',10,'MarkerFaceColor',[0.93 0.69 0.13]);
            hold all
            plot(goal_list(:,1),goal_list(:,2),'o','color',[0.93 0.69 0.13],'markersize',30)
            
            break;  %到达最后一个目标点跳出循环
        end
    
    end
%% 跳出循环的判定

        if( ~isempty(find(crash,1)) ); break;  end 
        if( step_goal > max_goalstep); break; end
        
%% 循环善后工作
        if(figure1), drawnow; end     %更新智能体运行图  准备下一个状态
%         pre_a = a;
    end
end