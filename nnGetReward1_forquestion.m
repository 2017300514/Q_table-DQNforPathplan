
function [reward, goal,decide_2,decide_3] = nnGetReward1_forquestion(action, prev_action, sensor2, sensor1, krasj,circle_plot1,circle_plot2,decide_2,decide_3)
%action ��С����ǰʱ�̵Ķ���
%pre_action��С����һʱ�̵Ķ���

%sensor1��С����ǰʱ����������������������ϰ���ľ���
%sensor2��С����һʱ����������������������ϰ���ľ���
% krasj ��¼С��״̬��Ϣ���ܹ�ͨ�����е�ֵ���ж�С���Ƿ���ײ�ϰ���
% circle_plot1  С����ǰʱ�̵�λ��
% circle_plot2  С����һʱ�̵�λ��
% decide_2  �ж�С���Ƿ���Ŀ��� 45dm��
% decide_3  �ж�С���Ƿ���Ŀ���22dm��

a = action;   %��¼С���Ķ�����ֱ��ȡ1����תȡ2����תȡ3
reward = 0;   %��ʼ������Ϊ0
goal = false; %��ʼ��Ĭ��С��û�е���Ŀ���
final_goal=[75,30];     %�趨Ŀ�������

dist=norm(final_goal-circle_plot1);    %���㵱ǰʱ��С����Ŀ������
dist1=norm(final_goal-circle_plot2);   %��¼��һʱ��С����Ŀ������
status = ~isempty(find(krasj(2),1));   %�ж�С���Ƿ��뻷���ϰ���߽���ײ

r = [0.01, -0.01, -0.9, 0.9];   %������������Ų�ͬ�Ľ���ֵ

switch(a)                       %�ж϶������������ֱ�У�����һֱ��ת��һֱ��ת
    case(1)
        reward=reward+r(1);
    case(2)
        reward=reward+r(2);
    case(3)
        reward=reward+r(2);
end

if(decide_2==0 && dist<=45)      %�������С����Ŀ��㿿��
    decide_2 = 1;
    reward = reward+0.5*r(4);
end
if(decide_2==1 && dist>45)
    decide_2 = 0;
    reward= reward- 0.5*r(4);
end

if(decide_3==0 && dist<=22)         %�������С��������Ŀ��㿿��
    decide_3 = 1;
    reward = reward+0.75*r(4);
end
if(decide_3==1 && dist>=22)
    decide_3 = 0;
    reward = reward-0.75*r(4);
end

if(decide_2==0)                %����Ϊ�˱���С��һֱ��Զ��Ŀ��������̽��
    reward=reward+r(2);
end
if(decide_3==0)                 %����Ϊ�˱���С��һֱ��Զ��Ŀ��������̽��
    reward=reward+r(2); 
end
if(decide_3==0 && dist1 > dist)
    reward=reward+r(1);
end
if(prev_action == 2 && a == 3)   %�������С������ҡ�Σ���ת��������ת
    
    reward = reward + 4*r(2);
elseif(prev_action == 3 && a == 2)  %�������С������ҡ�Σ���ת��������ת
    
    reward = reward + 4*r(2);

end


if(status)             %���С����ײ�ϰ������ͷ�������ѭ��
    
    reward = r(3);
end

if (sum(sensor1 - sensor2) >= 0)   %���ｱ��С���ܿ��ϰ��ʹ�ó��������������ϰ�������С
    
    reward = reward+ 3*r(1);
else
    
    reward = reward+ 3*r(2);
end
if(dist <= 2)           %�������ж�С������Ŀ��㸽�������轱��������ѭ��
    goal = true;
    disp(1);
    reward = reward+r(4);
end

end