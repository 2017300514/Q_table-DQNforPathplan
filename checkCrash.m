function [sensor, crash] = checkCrash(car, obstacle_polygon, obstacle_circle)
    %% 程序说明  检测撞击函数
    %{
    输入 智能体模型(包括主体和传感器) 多边形障碍物，和圆障碍物模型
    输出  传感器数值  和碰撞标记数组
    %}
    
    %% ----- Initialization  初始化参数

     A = get(car(1),'Vertices');     %Get vertices of the vehicle     获取车辆顶点坐标值
%     dist_crash = [1,1,1,1,1,1,1,1,1,1,0.3,0.3,0.3,0.3];               %Sensor distance (1.5 = 15cm) for crash    传感器探测撞击距离  15cm
    dist_crash = [1,1,1,1,1,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    obs_crash = false;              %Obstacle crash     障碍物撞击
    sens_crash = false;             %Crash sensor values are below 1.5.  探测距离低于1.5dm
    status = true;                  %Help variable       帮助变量

    %% ----- Check collision between the vehicle and obstacles
    
    n_polygons = length(obstacle_polygon);              %定义多边形障碍物的数量
    n_circles = length(obstacle_circle);                %定义圆形障碍物的数量
    for i = 1:n_polygons

        B = get(obstacle_polygon(i),'Vertices');        %获得多边形的所有顶点坐标值 

        obs_crash = obstacleCrash1(A, B, 'polygon');     %判断是否碰撞

        if(obs_crash)
            status = false; 
            break;                  
        end

    end

    if(status)    %判断是否碰撞圆形障碍物

        for i = 1:n_circles

            C = get(obstacle_circle(i),'Vertices');

            obs_crash = obstacleCrash1(A, C, 'circle');

            if(obs_crash)
                break;
            end
        end
    end

    %% ----- Check collision with sensors
    
    n_sensors = length(car(2:end));   %定义传感器的数量
    sensor = zeros(1, n_sensors);     % 初始化传感器数值
    
    % 遍历传感器
    for i = 1:n_sensors
        
           
        D = get(car(i+1),'Vertices');   %  获取传感器坐标
        sensor_temp = 0;
        
        % 对于每个传感器，判断所有圆形障碍物
        for j = 1:n_circles
            
            C = get(obstacle_circle(j), 'Vertices');
            
            % 在一个向量里储存所有的传感器数值
            [sensor_temp(j), crash] = sensorValues1(D, C, 'circle', dist_crash(i));
            
            % 检测碰撞
            if(crash)    %修改过，目的是为了减少碰撞的判定范围
                sens_crash = true;
            end
        end


        for k = (j+1):(j + n_polygons)   % 不能与圆形障碍物检测数值重叠，所以从j+1开始记录

            B = get(obstacle_polygon(k-j), 'Vertices');

            [sensor_temp(k), crash] = sensorValues1(D, B, 'polygon', dist_crash(i));

            if(crash)   

                sens_crash = true;
            end
        end
        
        % 记录最短的传感器数值
        sensor(i) = min(sensor_temp);

    end

    %sensor = round(10.*sensor);
    sensor = 10.*sensor;                % 乘以比例 1:10
    disp(sensor);
    crash = [obs_crash, sens_crash];    % 储存所有的碰撞结果
end