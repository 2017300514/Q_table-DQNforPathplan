
function reward = getReward2(action, pre_action, sensor, krasj,dist_a_obj,last_a_obj )
%function reward = getReward2(action, pre_action, sensor, krasj,dist_a_obj,last_a_obj ,dist_ave_limit)
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
%%

reward = 0;   %��ʼ������Ϊ 0

d_a_obsL = 100; %����ϰ���������� ��λ ����

d_a_obsR = 100; %�Ҳ��ϰ���������� ��λ ����  

status = ~isempty(find(krasj(2),1));   %�ж��������Ƿ��뻷���ϰ���߽���ײ  ������ײ�ж�Ϊ 0.15

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
elseif(status)             %�����������ײ�ϰ������ͷ�������ѭ��
    
    reward = reward_crash;
end

disp(reward);
end

