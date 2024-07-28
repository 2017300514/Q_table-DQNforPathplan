function [Q, tot_reward, i, getgoal,n_crash,n_arrive, info_s2,figure_z,pool,pool_num] = tableTrials(Q, alpha, gamma, T,stadium_option, n_arrive, tot_rot ,h_car,  ...
                                                                              h_circ, h_poly,state_space, action_list,max_steps,   ...
                                                                              n_crash,info_s1, h_text,init_place,goal_list,goal_num,figure_z,pool,pool_num)

global mode;
global figure1;

%% ----- Initialize parameters

v_car =  0.3;      % С�������ٶ� max ֱ���ٶ�

vector = 0.1;      % ��̬�ϰ����ƶ��ٶ�

pool_max =  10000;

sensor_lengde = 10;  

% ����Ҫ�����Ĳ�����ֻ��Ϊ�˳����ʼ��

getgoal = 0;       % �����ж��Ƿ񵽴�Ŀ��㣨�����㣩�Լ�����ֵ�жϵ���ڼ���Ŀ���


arrive_check = 0;

tot_reward = 0;    % �ܽ���

goal = goal_list(1,:);           % �������һ��Ŀ��λ��

path_x =  zeros(1,1000);         % ��ʼ���켣x���� ����
path_y =  zeros(1,1000);         % ��ʼ���켣y���� ����
path_x(1) = init_place(1);       % ��¼·�������x
path_y(1) = init_place(2);       % ��¼·�������y

rot_punkt= init_place;       % ������λ��

ang = table_cos_to_angle( h_car,rot_punkt,goal);  % ��ʼ�������

ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);

Q_temp = Q;                  % Q-table  ����ǰ��Q-table

sensor = checkCrash(h_car, h_poly, h_circ);         % ��ʼ��¼��������ֵ

state = discretization(state_space, sensor(1:5), stadium_option, ang);     % ��ʼ����ɢ״ֵ̬

dist_a_obj = norm(goal - rot_punkt); 

pool_copy = pool;

poolNum_copy = pool_num;

max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % ����ʼ�㵽��Ŀ������ಽ��

step_goal = 1;
%% ----- Trial with max_steps
    for i = 1:max_steps
%% ��ͣ����ģ��     
    if( strcmp(mode,'Pause1') )
%             i = 0;
        tot_reward = 0;
        Q = Q_temp;
        break;
    end

%% ��̬�ϰ����˶�ģ��
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
%% ��¼��һʱ��״̬���������
        last_a_obj = dist_a_obj;
        pre_rot_punkt = rot_punkt;           % ��һʱ��λ��
        temp_sensor = sensor;
        pre_ang_dqn = ang_dqn;
%%  Q-table ����Qֵ�����о���
       
        % a = tableEpsilonGreedyExploration(Q, state, action_list, epsilon);
        a = softMaxSelection(Q, state, action_list, T);

        step_goal = step_goal + 1;
        
        [tot_rot,rot_punkt]= doActionCircle(a, tot_rot, v_car,  h_car);
                                                                                    % [guide_goal,local_pos]=getCarposition(rot_punkt);
        dist_a_obj = norm(goal - rot_punkt);
        
        [sensor, crash] = checkCrash(h_car, h_poly, h_circ);                % �ж���ײ�ͻ�ȡ��������ֵ

        ang = table_cos_to_angle( h_car,rot_punkt,goal);                     %�������߽�
        
        [reward ,arrive_check]= getReward1(a,pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
        
        % ������       ���ø�
        [next_state, k1, k2, k3, k4] = discretization(state_space, sensor(1:5), stadium_option, ang);

        % ����Q��
        Q = updateQTable(state, a, reward, Q, next_state, alpha, gamma, arrive_check);
        
        state = next_state;
        % ����������ӵ�����
        ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);
        
        [reward_dqn,~] = nnGetReward1(a, pre_rot_punkt, rot_punkt, goal ,sensor, crash, dist_a_obj, last_a_obj);
%% ��¼���ݣ�����DQN����
        pool_count = poolNum_copy;                % ���������ݷ�����ӵ�λ��

        if poolNum_copy > pool_max                    % ������������ݳ���������������Ч���ݾ��ǳ��Ӵ�С           
           pool_count =  rem(pool_count,pool_max)+(rem(pool_count,pool_max)==0)*pool_max;
        end
        
        pool_copy(:,pool_count) = [sensor,ang_dqn,dist_a_obj,temp_sensor,pre_ang_dqn,last_a_obj,reward_dqn,arrive_check,a];
        
        poolNum_copy = poolNum_copy+1;
%% �����Ϣ����
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
        
%% ���ڵ���Ŀ����Ĵ���  
    if dist_a_obj < 2 
        n_arrive = n_arrive+1;                      % ��¼����Ŀ������
        getgoal = getgoal + 1 ;                     % ����֮����µ���Ŀ�����
        pool = pool_copy;
        pool_num = poolNum_copy;
        if getgoal < goal_num
            goal = goal_list(getgoal+1,:);            %  ����Ŀ�� ��Ŀ���йص�����Ҫ����
                       
            dist_a_obj = norm(goal-rot_punkt);      % ���¼�����һʱ�̾���Ŀ������
            
            ang = table_cos_to_angle( h_car,rot_punkt,goal);   %���߽�Ҳ��������
            
            state = discretization(state_space, sensor, stadium_option, ang);
            
            %����������ӵ�����ҲҪ����
            ang_dqn = cos_to_angle( h_car,rot_punkt,goal,sensor_lengde);
           
            max_goalstep = 2*(abs(goal(1) - rot_punkt(1))+ abs(goal(2) - rot_punkt(2)))/v_car;   % ����ʼ�㵽��Ŀ������ಽ��
            
            step_goal = 1;
        else
            figure_z = figure_z + 1;
            drawPath(figure_z, path_x(1:i+1), path_y(1:i+1),stadium_option);
            hold all
            plot(goal_list(:,1),goal_list(:,2),'d','color',[0.93 0.69 0.13],'markersize',10,'MarkerFaceColor',[0.93 0.69 0.13]);
            hold all
            plot(goal_list(:,1),goal_list(:,2),'o','color',[0.93 0.69 0.13],'markersize',30)
            
            break;  %�������һ��Ŀ�������ѭ��
        end
    
    end
%% ����ѭ�����ж�

        if( ~isempty(find(crash,1)) ); break;  end 
        if( step_goal > max_goalstep); break; end
        
%% ѭ���ƺ���
        if(figure1), drawnow; end     %��������������ͼ  ׼����һ��״̬
%         pre_a = a;
    end
end