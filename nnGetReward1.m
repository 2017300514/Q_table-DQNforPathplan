function [reward,arrive_check] = nnGetReward1(action, pre_rot_punkt, rot_punkt,goal ,sensor, krasj,dist_a_obj ,last_a_obj)

%% �������룬���˵��
%{
    ���룺  action          �����嵱ǰʱ�̵Ķ���

            pre_action      ��������һʱ�̵Ķ���

            sensor          �����嵱ǰʱ����������������������ϰ���ľ��� ��λcm

            krasj           ������״̬��Ϣ���ܹ�ͨ�����е�ֵ���ж��������Ƿ���ײ�ϰ���

            dist_a_o        �����嵱ǰʱ�̾���Ŀ������ ��λ��

            last_a_o        ��������һʱ�̾���Ŀ������ ��λ��

            dist_ave_limit  �����影�������ֶ����� �������ϰ�����룩��λ��
    ���:
            reward          ���������影��

    
%}

% ��������һ�� Ŀ���ת���� �� ��һʱ�������嵽Ŀ���������¶�������
%  ��������һ�� ���� �����ŵ�������������
%  ��������һ���ظ������������ϰ�����Ҳ�����ϰ�������� ��������discretization�����ã�����DQN��û����һ���̣�
%  ����ĸĶ��ǰ� ����������[-1��1]֮��
%%

reward = 0;   %��ʼ������Ϊ 0

arrive_check = 0;

status = ~isempty(find(krasj(2),1));   %�ж��������Ƿ��뻷���ϰ���߽���ײ  ������ײ�ж�Ϊ 0.15

% alpha = 1;
dist_x = abs(pre_rot_punkt(1)-goal(1))-abs(rot_punkt(1)-goal(1));

dist_y = abs(pre_rot_punkt(2)-goal(2))-abs(rot_punkt(2)-goal(2));

a = action;

reward_crash = -0.9;



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
    reward = reward + dist_reward;
    if dist_a_obj < 2
        reward = 0.9;
        arrive_check = 1;
    end
%     
else             %�����������ײ�ϰ������ͷ�������ѭ��
        
    reward = reward_crash;
end
    

end



