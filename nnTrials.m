function [nn_params,target_params, Q, Q_est, tot_reward, i,getgoal, n_crash, n_arrive,cost, info_s2,figure_z,pool,pool_num] = nnTrials(nn_params,target_params, input_layer_size, hidden_layer_size,...
                                                                                                                output_layer_size, alpha, gamma, T, actions, maxsteps,...
                                                                                                                n_crash, n_arrive,stadium_option,tot_rot, h_car, h_poly,  ...
                                                                                                                h_circ,info_s1,h_text,init_place,goal_list,goal_num,...
                                                                                                                figure_z,pool,pool_num)

%% ����˵�� DQN һ�� trial ��ȫ������ 
%{  
    ���룺  nn_params           ��������� 
            target_params      Ŀ�����������
            input_layer_size    �������Ԫ����  
            hidden_layer_size   ���ز���Ԫ����
            output_layer_size   �������Ԫ����
            alpha               Q-learning ���� alpha
            gamma               Q-learning ���� gamma
            T                   softmax ����T
            actions             �����б�
            maxsteps            һ��trial��߷��沽��
            n_crash             ��¼�ܵ���ײ����
            n_arrive            ��¼�ܵĵ������
            tot_rot             ��̬��
            stadium_option      ��ͼѡ��
            h_car               ������patch
            h_poly              ������ϰ���patch
            h_circ              Բ���ϰ���patch            
            info_s1             �����GUI����Ϣ
            h_text              ���ڵ���Ϣ����
            init_place          �������ʼ����
            figure_z            ��ͼ��ű���
    ���:   nn_params           ���������
            target_params       Ŀ�����������
            Q                   ����״̬��Qֵ  �������Ե�ʱ��鿴
            Q_est               ���ж�������һʱ�̵�����Qֵ
            reward              ���������影��

    
%}
%% ȫ�ֱ�������

global mode;
global figure2;


%% ----- Initialization
v_car = 0.3;                     % С�������ٶ� max ֱ���ٶ�

extract_num = 10;                % DQN ��ȡ��������

sensor_lengde = 10;              % ���������� ����ı�������ǹ�һ��

pool_max =  10000;

extract_step = 100;              % ����Ƶ�� ������ÿ�߶��ٲ�����һ������

vector = 0.1;                   % ��̬�ϰ����ƶ��ٶ�

% learn_time_set = 2;

% ����Ҫ�����Ĳ�����ֻ��Ϊ�˳����ʼ��

cost = 0;                        % ��¼���ۺ���֮��

a = 1 ;                          % ����

getgoal = 0;                     % �����ж��Ƿ񵽴�Ŀ��㣨�����㣩�Լ�����ֵ�жϵ���ڼ���Ŀ���

goal = goal_list(1,:);           % �������һ��Ŀ��λ��

% pre_goal =  goal;                % ����ǰһ��״̬��Ŀ��λ��

tot_reward = 0;                  % �ܽ���

rot_punkt = init_place;          % ������λ��

nn_params_temp = nn_params;      % ��ʷȨ�ؾ���  

path_x =  zeros(1,1000);         % ��ʼ���켣x���� ����
path_y =  zeros(1,1000);         % ��ʼ���켣y���� ����
path_x(1) = init_place(1);       % ��¼·�������x
path_y(1) = init_place(2);       % ��¼·�������y

dist_a_obj = norm(goal - rot_punkt);   % �������ʼ���룬Ŀ������ϰ����
% last_a_obj = dist_a_obj;               % �������ʼ���룬Ŀ������ϰ����

% max_dist = last_a_obj;


max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % ����ʼ�㵽��Ŀ������ಽ��

step_goal = 1;                  % ��¼����Ŀ���Ĳ���

ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);% ��ʼ�������

%num_iters = 1;             % �����漰��˫dqn����ע�ͱ���

sensor = checkCrash(h_car, h_poly, h_circ);         % ��ʼ��¼��������ֵ

% pool_size = size(pool);

nn_params_size = size(nn_params);

grad_pool = zeros(nn_params_size(1),extract_step);

%% ----- Trial with max_steps
for i = 1:maxsteps
%% ��ͣ����ģ��   
    if( strcmp(mode,'Pause2') )
%         i = 0;
        tot_reward = 0;
        nn_params = nn_params_temp;
        break;
    end
    
%% ��̬�ϰ����˶�ģ��
    if( strcmp(stadium_option, 'Dynamic') == 1 )
        rot1=atan(0.5);
        rot2=-rot1;
        if(mod(i,540)>270)                          % 270���� ������̬�ϰ��ﷴ�����ƶ� 
            rot1=atan(0.5)+pi;
            rot2=rot2+pi;
        end
        moveworker(h_circ(4),rot1,vector);
        moveworker(h_circ(5),rot2,vector);
    end

    
  
    
%% ��¼��һʱ��ץ״̬���������
    temp_sensor = sensor;                % ��һʱ�̴���������
    pre_ang = ang;                       % ��һʱ�̷���ǣ��ٶȺ�Ŀ�귽λ�н�
    pre_rot_punkt = rot_punkt;           % ��һʱ��λ��
    last_a_obj = dist_a_obj;
%% ��¼��һʱ���뽱���йص����� 
        % ���涯���������ʱ���漰�����ɱ�������
    pre_a=a;
    
%% DQN ����Qֵ�����о���
        % �����������ɵ�ǰ״̬��Qֵ
     

    Q = nnFeedForward(nn_params, input_layer_size, hidden_layer_size, ...
        output_layer_size, [sensor,ang,dist_a_obj],sensor_lengde);     
        % Select action with the exploration function  ʹ��̽������ѡȡ����
%     a = nnEpsilonGreedyExploration(Q, actions, epsilon);                      % ʹ�� epsilon ����ѡȡ����  ���������
   
    a = nnSoftMaxSelection(Q, actions, T);                                      % ʹ�� softmax ����ѡȡ����  ���������
    
    step_goal = step_goal + 1;                                                  % ���µ���Ŀ������õĲ���
    
    [tot_rot,rot_punkt]= doActionCircle(a, tot_rot, v_car,  h_car);             % ִ�о��ߣ��������ƶ�����������������
    
    dist_a_obj = norm(goal - rot_punkt);                                        % ������������������λ�� dm
    
    [sensor, crash] = checkCrash(h_car, h_poly, h_circ);                        % ���������崫����ֵ������һ״̬��
    
    ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);                    % ������һ״̬���߽�

    % ���㽱��������ִ�ж�������һ״̬�Ľ���
    [reward ,arrive_check] = nnGetReward2(a, pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
                                               
    % Calculate Q_estimat  ��������Qֵ

    Q_est = computeQEstimate(nn_params, input_layer_size, hidden_layer_size, ...
                            output_layer_size, [sensor,ang,dist_a_obj],reward,gamma,sensor_lengde,arrive_check);                        
    % Calculate the cost-function and the partial derivatives     ������ۺ�����ƫ����
    [C, grad] = nnBackPropagation(nn_params, input_layer_size, ...
                                  hidden_layer_size, output_layer_size, ...
                                  actions, [temp_sensor,pre_ang,last_a_obj] ,a, Q_est,sensor_lengde);

 %% DQN ��ȡ�������   
    
    

    
    pool_valid = pool_num;                % �����������Ч���ݵ����� 
    pool_count = pool_num;                % ���������ݷ�����ӵ�λ��
    
    if pool_num > pool_max                    % ������������ݳ���������������Ч���ݾ��ǳ��Ӵ�С           
       pool_valid = pool_max;
       pool_count =  rem(pool_count,pool_max)+(rem(pool_count,pool_max)==0)*pool_max;
    end
    
    pool(:,pool_count) = [sensor,ang,dist_a_obj,temp_sensor,pre_ang,last_a_obj,reward,arrive_check,a];                % ���ѧϰ����    
    
    pool_num = pool_num+1;                    % �Ѿ����������һ    
    


    % Update the weight matrices   ����Ȩ�ؾ���  ��������˳��

    mark = randchoose(1:pool_valid,extract_num);    %���ɴ�1 �� ������Ч���ݴ�С �� ��ȡ���� extract_num ����ͬ������� ���ȷֲ�
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


   
%% ������������Ϣ����
    path_x(i+1) = rot_punkt(1);
    path_y(i+1) = rot_punkt(2);
    if( ~isempty(find(crash(2),1)) ); n_crash = n_crash + 1; end  
    tot_reward = tot_reward + reward;     % �����ܽ���
    cost = cost + C;
    info_s2 = sprintf('%s%15.3f%17s%16.3f%14s%15.3f', 'Q1:', Q(1), 'Q2:', Q(2), 'Q3:', Q(3));
    info_s2 = sprintf('%s\n%s%9.3f%20s%14d%15s%15.6f', info_s2, 'Q_est:', Q_est, 'Crash:', n_crash, 'cost:', cost/i);
    info_s = sprintf('%s\n%s', info_s1, info_s2);
    set(h_text, 'String', info_s);       
%% ����Ŀ�������ݴ���
    if dist_a_obj < 2 
        n_arrive = n_arrive+1;                      % ��¼����Ŀ������
        getgoal = getgoal + 1 ;                     % ����֮����µ���Ŀ�����
        if getgoal < goal_num
            goal = goal_list(getgoal+1,:);            % ����Ŀ��
            
            dist_a_obj = norm(goal-rot_punkt);      % ���¼�����һʱ�̾���Ŀ������
                       
            ang = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde); 
            
            max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % ����ʼ�㵽��Ŀ������ಽ��

            step_goal = 1;                  % ��¼����Ŀ���Ĳ���
        
        else
            figure_z = figure_z + 1;  

            drawPath(figure_z, path_x(1:i+1), path_y(1:i+1),stadium_option);

            break;  %�������һ��Ŀ�������ѭ��
        end
            
    end
    
        

%% �˳�ѭ�������� ������ײ
    if( ~isempty(find(crash,1)) ); break; end
    if( step_goal > max_goalstep); break; end
    
    if(figure2), drawnow; end             % �������ͼ��׼��ѭ������һ��
end

cost = cost/i;

