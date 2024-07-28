function [tot_rot,rot_punkt] = doActionCircle(action, tot_rot,v_loit, h_car)

    % Function for moving the vehicle in the simulation
    %{ 
    输入： 动作序号，智能体目前的速度方向，大小，智能体的模型位置，包括小车模型和传感器模型
    输出： 小车的速度方向，和最新位置(模型主体圆的圆心)
    注意，theta 值输入到 rotate 函数，是matlab封装好的，
          正值逆时针，负值顺时针,单位是弧度，会在moveCarCircle.m里面转化为度；
    %}
    % 这个函数要做的改动是
    %{
    1. 变换运动空间，八个运动空间维度
    2. 改变运动速度和运动速度改变的方式，仍然用瞬间转移的方式进行
    3. 输出 方向角，位置
    %}


    v = v_loit;
    a = action;
    
    switch(a)
        
        % Move forward
        case 1
            theta = 0;
        
        % Turn left    
        case 2
            theta = pi/3;
        % Turn right
        case 3
            theta = -pi/3;
    end

    tot_rot = tot_rot + theta;          % Total rotation

    rot_punkt = moveCarCircle(v, tot_rot, theta, h_car(1));
    moveSensor(v, rot_punkt, tot_rot, theta, h_car(2:end)); 
end