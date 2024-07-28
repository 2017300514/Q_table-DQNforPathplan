function value = obstacle_angle( sensor,ob_punkt,sensor_punkt)  
    % 函数用来计算小车传感器与障碍物点夹角
    %  sensor  传感器中轴线向量
    %  ob_punkt 障碍物点
    value = false;                                   % 初始化判断目标点不再传感器范围内
    theta=30/180*pi;
%     A = get(sensor,'Vertices');                      % 获取传感器模型端点，
    w1 = sensor';              % 得到传感器检测扇形中轴线方向
    w2 = [ob_punkt(1)-sensor_punkt(1);ob_punkt(2)-sensor_punkt(2)];    % 小车圆心到目标点的距离向量
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));          % 计算arcos 得到角度的绝对值，判断在区域内即可
    if (ang <= theta)
        value = true;
    end
end
    