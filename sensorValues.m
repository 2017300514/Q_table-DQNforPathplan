function [sensor_value, crash] = sensorValues(sensor_vertices, obstacle, shape, dist_crash)

    %计算传感器检测目标点的距离
    %sensor_vertices  传感器patch 上点位置信息
    

    crash = false;

    P = obstacle;                   %   障碍物端点坐标
    Qx = sensor_vertices(:,1);      %    传感器测的x坐标   
    Qy = sensor_vertices(:,2);      %    传感器测的y坐标
    sensor_value = norm([Qx(end) - Qx(1), Qy(end) - Qy(1)]);    % 传感器的长度，默认没有检测到障碍物返回最大长度

    Qx_min = min(Qx);   %传感器模型的起点x             
    Qx_max = max(Qx);   %传感器模型的起点x

    Qy_min = min(Qy);  %传感器模型的起点y
    Qy_max = max(Qy);  %传感器模型的终点y

    n_points = 7000;
    x = linspace(Qx_min, Qx_max, n_points)';    %线段上均匀7000个点x坐标
    y = linspace(Qy_min, Qy_max, n_points)';    %线段上均匀7000个点y坐标
    n_x = length(x);        %    生成序列上点的坐标
    dist_between = 0.2;     %    传感器和障碍物之间允许的最小距离 用来检测碰撞

    %     计算传感器的方程（直线）
    b = Qy(end)- Qy(1);
    c = Qx(end)- Qx(1);

    if( c ~= 0 )
        a = b/c;                        %   计算斜率
        y = a.*(x - Qx(1)) + Qy(1);     %   直线方程  
    else
        n_points = length(y);
        x = Qx(1).*ones(n_points,1);
    end

    %检测圆形障碍物
    if( strcmp(shape,'circle') )  
        
        a = (min(P(:,1)) + max(P(:,1)))/2;
        b = (min(P(:,2)) + max(P(:,2)))/2;

        centre = [a, b];
        radius = norm([P(1,1) - centre(1), P(1,2) - centre(2)] );   %计算障碍物的半径


        f1 =  sqrt( radius.^2 - (x - a).^2 ) + b;
        f2 = -sqrt( radius.^2 - (x - a).^2 ) + b;
        
        %Sjekker differansen mellom sensorens funksjon y og hindringens funksjon f
        g1 = abs(y - f1);
        g2 = abs(y - f2);
        
        %检测碰撞并记录点的索引
        I1 = find( g1 < dist_between);
        I2 = find( g2 < dist_between);
        I = [I1; I2];
          
        if( ~isempty(I) )
            
            %    返回距离最近的点作为传感器的距离数值   这里有一个min函数
            sensor_value = min(dist([x(I), y(I)], [Qx(1); Qy(1)]));
            
            %检测碰撞
            if(sensor_value < dist_crash)

                crash = true;
            end

        end

    %  检测固态障碍物
    elseif( strcmp(shape,'polygon') )

        n_punkter = size(P,1);
        
        %f(x) = a*(x-x0) + f(x0) for kantene av hindringen
        for i = 1:n_punkter-1   % 这里计算的是n-1个边
  
            f = ones(n_x,1);
            g = ones(n_x,1);
            b = P(i+1, 2) - P(i, 2);
            c = P(i+1, 1) - P(i, 1);

            if( c ~= 0 )

                a = b/c;

                x_limit1 = min([P(i,1), P(i+1,1)]);     %x-min grense
                x_limit2 = max([P(i,1), P(i+1,1)]);     %x-maks grense

                %限制 传感器序列上的点在 边长端点x坐标范围内
                I = (x_limit1 < x) & (x < x_limit2);    

                f(I) = a.*(x(I) - P(i,1)) + P(i,2);     %funksjonen f(x)
                g(I) = abs(y(I) - f(I));                %diffensen mellom y og f(x)

            else   %  如果多边形边长是垂直的，返回横坐标差值即为距离

                y_limit1 = min([P(i,2), P(i+1,2)]);     %y-min grense
                y_limit2 = max([P(i,2), P(i+1,2)]);     %y-maks grense
            
                %   限制 传感器序列上的点在 边长端点y坐标范围内
                I = (y_limit1 < y) & (y < y_limit2);
                
                
                g(I) = abs( x(I) - P(i,1) );            

            end
            
            %记录最短的传感器数值，因为多边形有多个边，要检测多次，比较后保留最小值
            I = find( g < dist_between);
            
            if( ~isempty(I) )
                
                temp = min(dist([x(I), y(I)], [Qx(1); Qy(1)]));
                
                if(sensor_value > temp)
                    
                    sensor_value = temp;          
                end

                if(sensor_value < dist_crash)

                    crash = true;
                end
            end

        end
        
        
        %对于多边形障碍物最后一个边
        %hindringen   
        f = ones(n_x,1);
        g = ones(n_x,1);
        b = P(n_punkter, 2) - P(1, 2);
        c = P(n_punkter, 1) - P(1, 1);

        if(c ~= 0)

            a = b/c;

            x_limit1 = min([P(1,1), P(n_punkter,1)]);
            x_limit2 = max([P(1,1), P(n_punkter,1)]);

            I = (x_limit1 < x) & (x < x_limit2) ;

            f(I) = a*( x(I) - P(1,1)) + P(1,2);
            g(I) = abs(y(I) - f(I));


        else

            y_limit1 = min([P(1,2), P(n_punkter,2)]);
            y_limit2 = max([P(1,2), P(n_punkter,2)]);

            I = (y_limit1 < y) & (y < y_limit2) ;
            g(I) = abs( x(I) - P(1,1) );
        end

        I = find( g < dist_between);

        if( ~isempty(I) )

            temp = min(dist([x(I), y(I)], [Qx(1); Qy(1)]));
            
            if( sensor_value > temp)
                
                sensor_value = temp;
            end
            
            if(sensor_value < dist_crash)

                crash = true;
            end
        end
    end
    
end

%改进思路 

% 1.改y值加减为欧氏距离 直接算圆心距，使用平方做比较不要开方来减少计算量
% 2.计算到多边形的距离时，计算传感器起始点到线段的垂直距离，如果小于探测距离，判断距离传感器最近的点是否在探测区域，然后返回一个距离值