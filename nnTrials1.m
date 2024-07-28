function [nn_params, target_params, Q, Q_est, tot_reward, step, getgoal, n_crash, n_arrive, cost, info_s2, figure_z, pool, pool_num] = nnTrials1(nn_params, target_params, input_layer_size, hidden_layer_size,...
                                                                                                                  output_layer_size, alpha, gamma, T, actions, maxsteps,...
                                                                                                                  n_crash, n_arrive, stadium_option, tot_rot, h_car, h_poly,  ...
                                                                                                                  h_circ, info_s1, h_text, init_place, goal_list, goal_num,...
                                                                                                                  figure_z, pool, pool_num)
    %% 初始化
    global mode;
    global figure2;
    
    v_car = 0.3;
    extract_num = 10;
    sensor_lengde = 10;
    pool_max = 10000;
    extract_step = 100;
    vector = 0.1;
    
    cost = 0;
    action = 1;
    getgoal = 0;
    goal = goal_list(1,:);
    tot_reward = 0;
    rot_punkt = init_place;
    nn_params_temp = nn_params;
    
    path_x = zeros(1, 1000);
    path_y = zeros(1, 1000);
    path_x(1) = init_place(1);
    path_y(1) = init_place(2);
    
    dist_a_obj = norm(goal - rot_punkt);
    max_goalstep = 2 * (abs(goal(1) - rot_punkt(1)) + abs(goal(2) - rot_punkt(2))) / v_car;
    step_goal = 0;
    ang = cos_to_angle(h_car, rot_punkt, goal, sensor_lengde);
    sensor = checkCrash(h_car, h_poly, h_circ);
    
    nn_params_size = size(nn_params);
    grad_pool = zeros(nn_params_size(1), 1);

    %% 进行Trial
    for step = 1:maxsteps
        if strcmp(mode, 'Pause2')
            tot_reward = 0;
            nn_params = nn_params_temp;
            break;
        end
        
        if strcmp(stadium_option, 'Dynamic') == 1
            updateDynamicObstacles(h_circ, step, vector);
        end
        
        [temp_sensor, pre_ang, pre_rot_punkt, last_a_obj, pre_action] = recordPreviousState(sensor, ang, rot_punkt, dist_a_obj, action);
        
        Q = nnFeedForward(nn_params, input_layer_size, hidden_layer_size, output_layer_size, [sensor, ang, dist_a_obj], sensor_lengde);
        action = nnSoftMaxSelection(Q, actions, T);
        step_goal = step_goal + 1;
        [tot_rot, rot_punkt] = doActionCircle(action, tot_rot, v_car, h_car);
        dist_a_obj = norm(goal - rot_punkt);
        [sensor, crash] = checkCrash(h_car, h_poly, h_circ);
        ang = cos_to_angle(h_car, rot_punkt, goal, sensor_lengde);
        [reward, arrive_check] = nnGetReward2(action, pre_rot_punkt, rot_punkt, goal, sensor, crash, dist_a_obj, last_a_obj);
        
        Q_est = computeQEstimate(nn_params, input_layer_size, hidden_layer_size, output_layer_size, [sensor, ang, dist_a_obj], reward, gamma, sensor_lengde, arrive_check);
        [C, grad] = nnBackPropagation(nn_params, input_layer_size, hidden_layer_size, output_layer_size, actions, [temp_sensor, pre_ang, last_a_obj], action, Q_est, sensor_lengde);
        
        [pool,pool_valid] = updatePool(pool, pool_num, pool_max, [sensor, ang, dist_a_obj, temp_sensor, pre_ang, last_a_obj, reward, arrive_check, action]);
        pool_num = pool_num + 1;
        
        nn_params = updateNetworkWeights(nn_params, target_params, pool, pool_valid, extract_num, alpha, input_layer_size, hidden_layer_size, output_layer_size, actions, sensor_lengde);
        if rem(pool_num, extract_step) == 0
            target_params = nn_params;
        end
        
        path_x(step + 1) = rot_punkt(1);
        path_y(step + 1) = rot_punkt(2);
        
        if ~isempty(find(crash, 1))
            n_crash = n_crash + 1;
        end
        
        tot_reward = tot_reward + reward;
        cost = cost + C;
        info_s2 = updateInfoString(Q, Q_est, n_crash, cost, step);
        info_s = sprintf('%s\n%s', info_s1, info_s2);
        set(h_text, 'String', info_s);
        
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
        
                drawPath(figure_z, path_x(1:step+1), path_y(1:step+1),stadium_option);
        
                break;  %到达最后一个目标点跳出循环
            end
                
        end
        
        if ~isempty(find(crash, 1)) || step_goal > max_goalstep
            break;
        end
        
        if figure2
            drawnow;
        end
    end
    
    cost = cost / step;
end

% 添加辅助函数，保持主函数简洁
function updateDynamicObstacles(h_circ, step, vector)
    rot1 = atan(0.5);
    rot2 = -rot1;
    if mod(step, 540) > 270
        rot1 = atan(0.5) + pi;
        rot2 = rot2 + pi;
    end
    moveworker(h_circ(4), rot1, vector);
    moveworker(h_circ(5), rot2, vector);
end

function [temp_sensor, pre_ang, pre_rot_punkt, last_a_obj, pre_action] = recordPreviousState(sensor, ang, rot_punkt, dist_a_obj, action)
    temp_sensor = sensor;
    pre_ang = ang;
    pre_rot_punkt = rot_punkt;
    last_a_obj = dist_a_obj;
    pre_action = action;
end

function [pool,pool_valid] = updatePool(pool, pool_num, pool_max, data)
    pool_count = pool_num;
    pool_valid = pool_num;
    if pool_num > pool_max
        pool_count = rem(pool_count, pool_max) + (rem(pool_count, pool_max) == 0) * pool_max;
        pool_valid = pool_max;
    end
    pool(:, pool_count) = data;
end

% function nn_params = updateNetworkWeights(nn_params, target_params, pool,pool_valid, extract_num, alpha, input_layer_size, hidden_layer_size, output_layer_size, actions, sensor_lengde,grad_pool)
%     mark = randchoose(1:pool_valid, extract_num);
%     for count = 1:extract_num
%         data = pool(:, mark(count));
%         Q_est = computeQEstimate(target_params, input_layer_size, hidden_layer_size, output_layer_size, data(1:16)', data(33), 0.9, sensor_lengde, data(34));
%         [~, grad_pool] = nnBackPropagation(nn_params, input_layer_size, hidden_layer_size, output_layer_size, actions, data(17:32)', data(35), Q_est, sensor_lengde);
%         nn_params = gradientDescent(nn_params, grad_pool, alpha, input_layer_size, hidden_layer_size, output_layer_size);
%     end
%     % grad_average = mean(grad_pool, 2);
% 
% end
function nn_params = updateNetworkWeights(nn_params, target_params, pool,pool_valid, extract_num, alpha, input_layer_size, hidden_layer_size, output_layer_size, actions, sensor_lengde)
    mark = randchoose(1:pool_valid, extract_num);
    for count = 1:extract_num
        data = pool(:, mark(count));
        Q_est = computeQEstimate(target_params, input_layer_size, hidden_layer_size, output_layer_size, data(1:16)', data(33), 0.9, sensor_lengde, data(34));
        [~, grad_pool(:, count)] = nnBackPropagation(nn_params, input_layer_size, hidden_layer_size, output_layer_size, actions, data(17:32)', data(35), Q_est, sensor_lengde);
    end
    grad_average = mean(grad_pool, 2);
    nn_params = gradientDescent(nn_params, grad_average, alpha, input_layer_size, hidden_layer_size, output_layer_size);
end
function info_s2 = updateInfoString(Q, Q_est, n_crash, cost, step)
    info_s2 = sprintf('%s%15.3f%17s%16.3f%14s%15.3f', 'Q1:', Q(1), 'Q2:', Q(2), 'Q3:', Q(3));
    info_s2 = sprintf('%s\n%s%9.3f%20s%14d%15s%15.6f', info_s2, 'Q_est:', Q_est, 'Crash:', n_crash, 'cost:', cost / step);
end


