function [goal_point,local_position]=getCarposition(circle_center)
    %% ����˵���� ��������ж�С�����ڵĲ���λ�ú�Ŀ��㣬��Ϊ��������������������������
    %{
       ����  Q��       Qֵ��
             state:    ״̬
             actions   �����б�
       ���  Q_value   ���Qֵ
             a         ���Qֵ��Ӧ�Ķ���
    %}
    
    x = circle_center(1);
    y = circle_center(2);

    if(x<=25&&x>=10&&y<=45)
        local_position=1;
        dist = norm(circle_center-[17.5,45]);
        if(dist>3)
            goal_point = [17.5,45];
        else 
            goal_point = [80,45];
        end 
    elseif (y>45&&y<65)
        local_position= 2;
        dist = norm(circle_center-[75,30]);
        if(dist>3)
            goal_point = [75,30];
        else 
            goal_point = [80,45];
        end 
    elseif (x<=25&&x>=10&&y>=65)
        local_position = 3;
        dist = norm(circle_center-[17.5,65]);
        if(dist>3)
            goal_point = [17.5,65];
        else 
            goal_point = [80,45];
        end 

    elseif (x>=70&&y<=45)
        local_position = 4;
        goal_point = [75,30];
    end
        
end