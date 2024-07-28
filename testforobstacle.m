

% [X,Y] = circle([10, 46], 1, 100);
% cir4_1 = patch(X,Y,'r','EdgeColor', 'b');
% car = get(cir4_1, 'Vertices');
% [X,Y] = circle1([90, 40], 5,[pi,1.5*pi], 200);
% cir3_1 = patch(X,Y,'r','EdgeColor', 'b');
% obstacle = get(cir3_1, 'Vertices');
%dist_crash  传感器碰撞检测范围
obstacle=[0,0,12.5,12.5,10,10,25,25,22.5,22.5,90,90,85,85,90,90,70,70,75,75,22.5,22.5,25,25,10,10,12.5,12.5;45,65,65,75,75,100,100,75,75,65,65,45,45,40,40,15,15,40,40,45,45,40,40,0,0,40,40,45];
obstacle = obstacle';
% obstacle =[10, 85; 10, 90; 16, 87.5];
% function crash = obstacleCrash1(car, obstacle, shape)

%检测碰撞障碍物的函数，返回一个布尔值
%car 小车模型的顶点坐标
%obstacle  障碍物顶点坐标
% shape    障碍物形状

crash = false;          %    初始化碰撞判断为无
shape = 'polygon';
P = obstacle;           %    障碍物顶点坐标
x = car(:,1);           %    小车顶点 x 坐标
y = car(:,2);           %    小车顶点 y 坐标
dist_between = 0.5;     %    规定的碰撞距离
n_punkter = size(car,1);
x0 = car(n_punkter,1);
y0 = car(n_punkter,2);
dist_value=10;

centre = [x0, y0];    %计算小车圆心位置
radius = norm([car(1,1) - x0, car(1,2) - y0] );  %计算小车半径
min_dist=radius+dist_between;
%最近距离是 规定距离加上半径，因为这里计算距离都是代入圆心的

% 判断障碍物形状
if( strcmp(shape,'circle') )
    n_punkter = size(P,1);
    a1 = P(n_punkter,1);
    b1 = P(n_punkter,2);

    
    radius_obs = norm([P(1,1) - a1, P(1,2) - b1] );  %计算圆形障碍物半径
    
    centre_dist = (x0-a1)^2+(y0-b1)^2;   %计算圆心距
    
    
    if( centre_dist<(radius_obs+radius+dist_between)^2)
        %如果圆心距小于半径之和加碰撞距离，则判定为碰撞
        crash = true;
    end
    
    %  对于多边形障碍物  如果垂足在障碍物边上，返回垂直距离，否则返回 距离两个端点的最近距离
elseif( strcmp(shape,'polygon') )
    
    n_punkter = size(P,1);     %多边形顶点数量

    for i = 1:n_punkter-1   % 这里计算的是n-1个边
        b = P(i+1, 2) - P(i, 2);
        c = P(i+1, 1) - P(i, 1);
        if( abs(c) < 0.001 );kkp_value = false; else; kkp_value = true;  kk_p  = b/c; b_p = -kk_p * P(i, 1) + P(i, 2);         end
        
        if( abs(b) < 0.001 );kkc_value = false; else; kkc_value = true;  kk_c=-c/b;  b_c = -kk_c * x0 + y0;           end
        
        x_limit1 = min([P(i,1), P(i+1,1)]);     % 边长的左端点x坐标
        x_limit2 = max([P(i,1), P(i+1,1)]);     % 边长的右端点x坐标
        y_limit1 = min([P(i,2), P(i+1,2)]);     % 边长的下面端点y坐标
        y_limit2 = max([P(i,2), P(i+1,2)]);     % 边长的上面端点y坐标   这几个量用于下面判断垂足是否在边长上
        vertice1=[P(i,1),P(i,2)];
        vertice2=[P(i+1,1),P(i+1,2)];          %记录两个端点坐标
        if(~kkc_value)
            dist_patch = (y0 - P(i,2))^2;
        elseif(~kkp_value)
            dist_patch = (x0-P(i, 1))^2;
        else                 %如果多边形边长是垂直的，返回x坐标的差的平方
            dist_patch = dist_pointToline(kk_p,b_p,centre);     %计算垂直距离的平方，用点到直线距离公式
        end
        %         if(kkp_value)
        %             dist_patch = dist_pointToline(kk_p,b_p,centre);     %计算垂直距离的平方，用点到直线距离公式
        %         else
        %             dist_patch = (x0-P(i, 1))^2;                 %如果多边形边长是垂直的，返回x坐标的差的平方
        %         end
        if(~kkc_value)
            x_vertical = x0;    %如果垂线无斜率，返回圆心横坐标
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %如果垂足在边长范围内，返回垂线段长度，否则返回到达最近顶点的距离
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
            end
            
        elseif(~kkp_value)
            y_vertical = y0;    %  如果多边形边长垂直x轴， 这里计算垂足纵坐标
            if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
            end
        else
            x_vertical = (b_c-b_p)/(kk_p-kk_c);     %返回垂足的横坐标
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)   %如果交点坐标在范围内，返回垂直距离
                dist_temp = sqrt(dist_patch);
            else
                dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
                %如果交点坐标不在范围内，返回最近的端点距离
            end
        end
        
        
        if(dist_value>dist_temp); dist_value=dist_temp;end    % 保留最短的距离值、
    end
    
    
    
    %对于多边形障碍物最后一个边
    
    b = P(n_punkter, 2) - P(1, 2);
    c = P(n_punkter, 1) - P(1, 1);
    if( abs(c) < 0.001 );kkp_value = false; else; kkp_value = true;  kk_p  = b/c;  b_p = -kk_p * P(1, 1) + P(1, 2);       end
    if( abs(b) < 0.001 );kkc_value = false; else; kkc_value = true;  kk_c=  -c/b;  b_c = -kk_c * x0 + y0;           end
    x_limit1 = min([P(1,1), P(n_punkter,1)]);     % 边长的左端点x坐标
    x_limit2 = max([P(1,1), P(n_punkter,1)]);     % 边长的右端点x坐标
    y_limit1 = min([P(1,2), P(n_punkter,2)]);     % 边长的下面端点y坐标
    y_limit2 = max([P(1,2), P(n_punkter,2)]);     % 边长的上面端点y坐标   这几个量用于下面判断垂足是否在边长上
    vertice1=[P(1,1),P(1,2)];                     %端点1 坐标
    vertice2=[P(n_punkter,1),P(n_punkter,2)];     %端点2 坐标
    
    
    if(~kkc_value)
        dist_patch = (y0 - P(1,2))^2;
    elseif(~kkp_value)
        dist_patch = (x0-P(1, 1))^2;
    else                 %如果多边形边长是垂直的，返回x坐标的差的平方
        dist_patch = dist_pointToline(kk_p,b_p,centre);     %计算垂直距离的平方，用点到直线距离公式
    end
    if(~kkc_value)
        x_vertical = x0;     %如果垂线无斜率，返回圆心横坐标
        
        if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %如果垂足在边长范围内，返回垂线段长度，否则返回到达最近顶点的距离
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
        
    elseif(~kkp_value)
        y_vertical = y0;    %  如果多边形边长垂直x轴， 这里计算垂足纵坐标
        if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
    else
        x_vertical = (b_c-b_p)/(kk_p-kk_c);
        if(x_vertical>=x_limit1 && x_vertical<=x_limit2)
            dist_temp = sqrt(dist_patch);
        else
            dist_temp = min([pdist([vertice1;centre],'euclidean'),pdist([vertice2;centre],'euclidean')]);
        end
    end
    
    
    if(dist_value>dist_temp); dist_value=dist_temp;end    % 保留最短的距离值
    
    
    if(dist_value< min_dist)
        crash = true;
    end
    
end


