function [nn_params,target_params, Q, Q_est, tot_reward, i,getgoal, n_crash, n_arrive,cost, info_s2,figure_z,pool,pool_num] = nnTrials(nn_params,target_params, input_layer_size, hidden_layer_size,...
                                                                                                                output_layer_size, alpha, gamma, T, actions, maxsteps,...
                                                                                                                n_crash, n_arrive,stadium_option,tot_rot, h_car, h_poly,  ...
                                                                                                                h_circ,info_s1,h_text,init_place,goal_list,goal_num,...
                                                                                                                figure_z,pool,pool_num)

%% 程序说明 DQN 一个 trial 的全部流程 
%{  
    输入：  nn_params           神经网络参数 
            target_params      目标神经网络参数
            input_layer_size    输入层神经元个数  
            hidden_layer_size   隐藏层神经元个数
            output_layer_size   输出层神经元个数
            alpha               Q-learning 参数 alpha
            gamma               Q-learning 参数 gamma
            T                   softmax 参数T
            actions             动作列表
            maxsteps            一个trial最高仿真步数
            n_crash             记录总的碰撞次数
            n_arrive            记录总的到达次数
            tot_rot             姿态角
            stadium_option      地图选项
            h_car               智能体patch
            h_poly              多边形障碍物patch
            h_circ              圆形障碍物patch            
            info_s1             输出到GUI的信息
            h_text              窗口的信息变量
            init_place          智能体初始坐标
            figure_z            画图标号变量
    输出:   nn_params           神经网络参数
            target_params       目标神经网络参数
            Q                   当下状态的Q值  用来调试的时候查看
            Q_est               进行动作后下一时刻的最优Q值
            reward              给予智能体奖励

    
%}
%% 全局变量定义

global mode;
global figure2;


%% ----- Initialization
v_car = 0.3;                     % 小车运行速度 max 直行速度

extract_num = 10;                % DQN 抽取样本数量

sensor_lengde = 10;              % 传感器长度 这个改变调整的是归一化

pool_max =  10000;

extract_step = 100;              % 更新频率 智能体每走多少步更新一次网络

vector = 0.1;                   % 动态障碍物移动速度

% learn_time_set = 2;

% 不需要调整的参数，只是为了程序初始化

cost = 0;                        % 记录代价函数之和

a = 1 ;                          % 动作

getgoal = 0;                     % 用来判定是否到达目标点（引导点）以及根据值判断到达第几个目标点

goal = goal_list(1,:);           % 智能体第一个目标位置

% pre_goal =  goal;                % 定义前一个状态的目标位置

tot_reward = 0;                  % 总奖励

rot_punkt = init_place;          % 智能体位置

nn_params_temp = nn_params;      % 历史权重矩阵  

path_x =  zeros(1,1000);         % 初始化轨迹x坐标 向量
path_y =  zeros(1,1000);         % 初始化轨迹y坐标 向量
path_x(1) = init_place(1);       % 记录路径坐标的x
path_y(1) = init_place(2);       % 记录路径坐标的y

dist_a_obj = norm(goal - rot_punkt);   % 这个是起始距离，目标点与障碍物的
% last_a_obj = dist_a_obj;               % 这个是起始距离，目标点与障碍物的

% max_dist = last_a_obj;


max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % 从起始点到达目标点的最多步数

step_goal = 1;                  % 记录到达目标点的步数

ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);% 初始化方向角

%num_iters = 1;             % 可能涉及到双dqn，先注释保留

sensor = checkCrash(h_car, h_poly, h_circ);         % 初始记录传感器的值

% pool_size = size(pool);

nn_params_size = size(nn_params);

grad_pool = zeros(nn_params_size(1),extract_step);

%% ----- Trial with max_steps
for i = 1:maxsteps
%% 暂停仿真模块   
    if( strcmp(mode,'Pause2') )
%         i = 0;
        tot_reward = 0;
        nn_params = nn_params_temp;
        break;
    end
    
%% 动态障碍物运动模块
    if( strcmp(stadium_option, 'Dynamic') == 1 )
        rot1=atan(0.5);
        rot2=-rot1;
        if(mod(i,540)>270)                          % 270步后 两个动态障碍物反方向移动 
            rot1=atan(0.5)+pi;
            rot2=rot2+pi;
        end
        moveworker(h_circ(4),rot1,vector);
        moveworker(h_circ(5),rot2,vector);
    end

    
  
    
%% 记录上一时刻抓状态输入的数据
    temp_sensor = sensor;                % 上一时刻传感器数据
    pre_ang = ang;                       % 上一时刻方向角，速度和目标方位夹角
    pre_rot_punkt = rot_punkt;           % 上一时刻位置
    last_a_obj = dist_a_obj;
%% 记录上一时刻与奖励有关的数据 
        % 后面动作数量多的时候涉及动作成本的问题
    pre_a=a;
    
%% DQN 产生Q值，进行决策
        % 从神经网络生成当前状态的Q值
     

    Q = nnFeedForward(nn_params, input_layer_size, hidden_layer_size, ...
        output_layer_size, [sensor,ang,dist_a_obj],sensor_lengde);     
        % Select action with the exploration function  使用探索函数选取动作
%     a = nnEpsilonGreedyExploration(Q, actions, epsilon);                      % 使用 epsilon 函数选取动作  带有随机性
   
    a = nnSoftMaxSelection(Q, actions, T);                                      % 使用 softmax 函数选取动作  带有随机性
    
    step_goal = step_goal + 1;                                                  % 更新到达目标点所用的步数
    
    [tot_rot,rot_punkt]= doActionCircle(a, tot_rot, v_car,  h_car);             % 执行决策，智能体移动，更新智能体坐标
    
    dist_a_obj = norm(goal - rot_punkt);                                        % 距离计量函数，输出单位是 dm
    
    [sensor, crash] = checkCrash(h_car, h_poly, h_circ);                        % 更新智能体传感器值，是下一状态的
    
    ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);                    % 计算下一状态视线角

    % 计算奖励，这是执行动作后下一状态的奖励
    [reward ,arrive_check] = nnGetReward2(a, pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
                                               
    % Calculate Q_estimat  计算最优Q值

    Q_est = computeQEstimate(nn_params, input_layer_size, hidden_layer_size, ...
                            output_layer_size, [sensor,ang,dist_a_obj],reward,gamma,sensor_lengde,arrive_check);                        
    % Calculate the cost-function and the partial derivatives     计算代价函数和偏导数
    [C, grad] = nnBackPropagation(nn_params, input_layer_size, ...
                                  hidden_layer_size, output_layer_size, ...
                                  actions, [temp_sensor,pre_ang,last_a_obj] ,a, Q_est,sensor_lengde);

 %% DQN 抽取数据设计   
    
    

    
    pool_valid = pool_num;                % 定义池子内有效数据的数量 
    pool_count = pool_num;                % 定义新数据放入池子的位置
    
    if pool_num > pool_max                    % 如果进来的数据超过池子容量，有效数据就是池子大小           
       pool_valid = pool_max;
       pool_count =  rem(pool_count,pool_max)+(rem(pool_count,pool_max)==0)*pool_max;
    end
    
    pool(:,pool_count) = [sensor,ang,dist_a_obj,temp_sensor,pre_ang,last_a_obj,reward,arrive_check,a];                % 填充学习池子    
    
    pool_num = pool_num+1;                    % 已经填充数量加一    
    


    % Update the weight matrices   更新权重矩阵  按照周期顺序

    mark = randchoose(1:pool_valid,extract_num);    %生成从1 到 池子有效数据大小 的 抽取数量 extract_num 个不同的随机数 均匀分布
    for count = 1:extract_num
        pool_extract = pool(:,mark(count));
        sensor_num = pool_extract(1:14);
        ang_num = pool_extract(15);
        dist_num =  pool_extract(16);
        temp_sensor_num = pool_extract(17:30);
        pre_ang_num = pool_extract(31);
        lastd_num = pool_extract(32);
        reward_num = pool_extract(33);
        arriveChar_num = pool_extract(34);
        action_num = pool_extract(35);

        Q_est = computeQEstimate(target_params, input_layer_size, hidden_layer_size, ...
                                output_layer_size, [sensor_num',ang_num,dist_num],reward_num,gamma,sensor_lengde,arriveChar_num);
        [C, grad_pool(:,count)] = nnBackPropagation(nn_params, input_layer_size, ...
                                      hidden_layer_size, output_layer_size, ...
                                      actions, [temp_sensor_num',pre_ang_num,lastd_num] ,action_num, Q_est,sensor_lengde);
    end
    grad_average =  mean(grad_pool,2); 
    nn_params = gradientDescent(nn_params, grad_average, alpha, ...
                            input_layer_size, hidden_layer_size, ...
                            output_layer_size);
    
    if (rem(pool_count,extract_step) == 0)
        target_params = nn_params;
    end


   
%% 输出到界面的信息整理
    path_x(i+1) = rot_punkt(1);
    path_y(i+1) = rot_punkt(2);
    if( ~isempty(find(crash(2),1)) ); n_crash = n_crash + 1; end  
    tot_reward = tot_reward + reward;     % 计算总奖励
    cost = cost + C;
    info_s2 = sprintf('%s%15.3f%17s%16.3f%14s%15.3f', 'Q1:', Q(1), 'Q2:', Q(2), 'Q3:', Q(3));
    info_s2 = sprintf('%s\n%s%9.3f%20s%14d%15s%15.6f', info_s2, 'Q_est:', Q_est, 'Crash:', n_crash, 'cost:', cost/i);
    info_s = sprintf('%s\n%s', info_s1, info_s2);
    set(h_text, 'String', info_s);       
%% 到达目标点后数据处理
    if dist_a_obj < 2 
        n_arrive = n_arrive+1;                      % 记录到达目标点次数
        getgoal = getgoal + 1 ;                     % 到达之后更新到达目标序号
        if getgoal < goal_num
            goal = goal_list(getgoal+1,:);            % 更新目标
            
            dist_a_obj = norm(goal-rot_punkt);      % 重新计算上一时刻距离目标点距离
                       
            ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde); 
            
            max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % 从起始点到达目标点的最多步数

            step_goal = 1;                  % 记录到达目标点的步数
        
        else
            figure_z = figure_z + 1;  

            drawPath(figure_z, path_x(1:i+1), path_y(1:i+1),stadium_option);

            break;  %到达最后一个目标点跳出循环
        end
            
    end
    
        

%% 退出循环的条件 发生碰撞
    if( ~isempty(find(crash,1)) ); break; end
    if( step_goal > max_goalstep); break; end
    
    if(figure2), drawnow; end             % 更新输出图，准备循环的下一步
end

cost = cost/i;

