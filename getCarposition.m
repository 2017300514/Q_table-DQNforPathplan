function [goal_point,local_position]=getCarposition(circle_center)
    %% 函数说明： 这个用来判断小车所在的舱室位置和目标点，作为奖励函数的输入和神经网络的输入
    %{
       输入  Q：       Q值表
             state:    状态
             actions   动作列表
       输出  Q_value   最佳Q值
             a         最佳Q值对应的动作
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