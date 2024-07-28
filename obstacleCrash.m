function crash = obstacleCrash(car, obstacle, shape)
    
    %检测碰撞障碍物的函数，返回一个布尔值
    %car 小车模型的顶点坐标
    %obstacle  障碍物顶点坐标
    % shape    障碍物形状
  	crash = false;
    
	P = obstacle;           %    障碍物顶点坐标
	x = car(:,1);           %   小车顶点 x 坐标
	y = car(:,2);           %   小车顶点 y 坐标
    dist_between = 0.3;     
    
    % 判断障碍物形状
    if( strcmp(shape,'circle') )
                
		a = (min(P(:,1)) + max(P(:,1)))/2;
        b = (min(P(:,2)) + max(P(:,2)))/2; 
        
        centre = [a, b];    %如果是圆形障碍物，通过取平均计算圆心位置
        radius = norm([P(1,1) - centre(1), P(1,2) - centre(2)] );  %计算半径
        
        %计算所有小车顶点横坐标对应圆上的y坐标，因为圆对称性得到两个
		f1 =  sqrt( radius.^2 - (x - a).^2 ) + b;
		f2 = -sqrt( radius.^2 - (x - a).^2 ) + b;  %sqrt代入负数得到虚数
        
        % 得到到两个对称点的纵向距离
		g1 = abs(y-f1);
		g2 = abs(y-f2);   %代入虚数得到模长  
        
        %圆
        if( ~isempty(find( g1 < dist_between, 1)) || ~isempty(find( g2 < dist_between, 1)) )
            
            crash = true;
        end

    %  对于多边形障碍物
    elseif( strcmp(shape,'polygon') )

		n_punkter = size(P,1);
        
        % f(x) = a*(x-x0) + f(x0)   计算多边形 的n-1条边
        for i = 1:n_punkter-1  
            
            g = [];
            b = P(i+1, 2) - P(i, 2);
            c = P(i+1, 1) - P(i, 1);
            
            if( c ~= 0 )
            
                a = b/c;

                x_limit1 = min([P(i,1), P(i+1,1)]);         %线段左端点
                x_limit2 = max([P(i,1), P(i+1,1)]);         %线段右端点
                
                
                I = find((x_limit1 < x) & (x < x_limit2) );     %将顶点限制在两个端点之间的横坐标之间，并记录判断值
                
                if( ~isempty(I) )
                    
                    f = a.*(x(I) - P(i,1)) + P(i,2);        %function f(x)
                    g = abs(y(I) - f);                      % 计算y距离值
                end

            else    %如果斜率无穷大，即多边形边长垂直x轴 返回x坐标差即为距离
                
                y_limit1 = min([P(i,2), P(i+1,2)]);         %y-min grense
                y_limit2 = max([P(i,2), P(i+1,2)]);         %y-maks grense
                
                % 将顶点限制在两个端点之间的竖直距离之间，并记录判断值
                I = find( (y_limit1 < y) & (y < y_limit2) );    
                
                if( ~isempty(I) )
                    
                    
                    g = abs( x(I) - P(i,1) );
                end
                
            end
			
            % 判断距离是否达到碰撞
            if( ~isempty(find( g < dist_between, 1)) )
                    
                crash = true;
                return;
            end
            
        end
        % 计算多边形的最后一个边长
        g = [];
        b = P(n_punkter, 2) - P(1, 2);
        c = P(n_punkter, 1) - P(1, 1);
        
        if(c ~= 0)
            
            a = b/c;
            
            x_limit1 = min([P(1,1), P(n_punkter,1)]);
            x_limit2 = max([P(1,1), P(n_punkter,1)]);
            
            I = find((x_limit1 < x) & (x < x_limit2) );
            
            if( ~isempty(I) )
                
                f = a*( x(I) - P(1,1)) + P(1,2);
                
                g = abs(y(I) - f);
            end
                    
        else      
            
            y_limit1 = min([P(1,2), P(n_punkter,2)]);
            y_limit2 = max([P(1,2), P(n_punkter,2)]);
            
            I = find( (y_limit1 < y) & (y < y_limit2) );
            
            if( ~isempty(I) )
                
                g = abs( x(I) - P(1,1) );
            end
            
        end
        
        if( ~isempty(find( g < dist_between, 1)) )

			crash = true;
        end
    end
end
