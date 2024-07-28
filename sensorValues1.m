function [sensor_value, crash] = sensorValues1(sensor_vertices, obstacle, shape, dist_crash)

%计算传感器检测目标点的距离
%sensor_vertices  传感器patch
%obstacle   障碍物 patch
%shape   障碍物种类

%dist_crash  传感器碰撞检测范围



crash = false;

% k_value=true;               % 初始化三条直线的斜率状态，代表有斜率
kk1_value=true;
kk2_value=true;

%     dist1=0;                     %传感器检测最短距离dist1,dist2
%     dist2=0;
theta1=pi/6;
theta2=2*pi-theta1;
kk2=0;                      %定义kk2是对应逆时针旋转三十度的直线，方程的斜率
kk1=0;                      %定义kk1是对应顺时针旋转三十度的直线，方程的斜率
P = obstacle;                   %   障碍物端点坐标
Qx = sensor_vertices(:,1);      %    传感器模型端点的x坐标
Qy = sensor_vertices(:,2);      %    传感器模型端点的y坐标
sensor_vector= [Qx(end) - Qx(1), Qy(end) - Qy(1)];
Q_coord=[Qx(1),Qy(1)];
sensor_length = norm(sensor_vector);    % 传感器探测最大距离
sensor_value = sensor_length;           % 默认没有检测到障碍物返回最大距离
%     计算传感器的方程（直线）
b = Qy(end)- Qy(1);
c = Qx(end)- Qx(1);    %用来计算传感器直线方程
if( abs(c) > 0.001 )
    k=b/c;
    if( abs(k-1.732)<= 0.001 )    % 中轴线倾角在六十度附近，逆时针旋转30度没有斜率
        kk2_value=false;
        kk1 = (k*cos(theta2) + sin(theta2)) / (cos(theta2) - k*sin(theta2));
        b1 = Qy(1)-kk1*Qx(1);
    elseif( abs(k+1.732)<= 0.001 )
        kk1_value=false;
        kk2 = (k*cos(theta1) + sin(theta1)) / (cos(theta1) - k*sin(theta1));
        b2 = Qy(1)-kk2*Qx (1);
    else
        kk1 = (k*cos(theta2) + sin(theta2)) / (cos(theta2) - k*sin(theta2));
        kk2 = (k*cos(theta1) + sin(theta1)) / (cos(theta1) - k*sin(theta1));
        b1 = Qy(1)-kk1*Qx(1);         % 扇形顺时针边界线2 直线方程  y=kk2*x+b2 点斜式
        b2 = Qy(1)-kk2*Qx(1);         % 扇形逆时针边界线1 直线方程  y=kk1*x+b1 点斜式
    end
else                                  % 如果传感器中轴线垂直x轴，k不存在，直接返回两个旋转斜率
    kk2=-1.732;                       %  kk2是逆时针的斜率
    kk1=+1.732;                       %  kk1是逆时针的斜率
    b2 = Qy(1)-kk2*Qx(1);
    b1 = Qy(1)-kk1*Qx(1);
end

%检测圆形障碍物
if( strcmp(shape,'circle') )
    
    n_punkter = size(P,1);
    a = P(n_punkter,1);
    b = P(n_punkter,2);
    centre = [a, b];
    radius = norm([P(1,1) - a, P(1,2) - b] );   %计算障碍物的半径
    dist_real=pdist([centre;Q_coord],'euclidean');                          %障碍物与传感器圆心距
    deta_theta=asin(radius/dist_real);
    help_1=sensor_vector'; 
    dist_help=(centre-Q_coord)';
    help_2 =cross([help_1;0],[dist_help;0]);                                      
    
    theta = acos(dist_help'*help_1/(norm(dist_help)*norm(help_1)));  % 计算圆心连线与传感器向量的夹角
    if (help_2(3)<0)
        theta = -theta;
    end
   
    lamda2=theta+deta_theta;
    lamda1=theta-deta_theta;   % lamda是靠近扇形边界区域那一侧的切线与扇形边的夹角
                                  
    if(dist_real<=sensor_length+radius&&(~(lamda1>pi/6||lamda2<-pi/6)))                               %如果障碍物进入可能检测范围, d-r<探测距离
        if(obstacle_angle( sensor_vector,centre,Q_coord) )   %如果障碍物圆心在检测角度内，返回最短距离=d-r
            sensor_value = dist_real-radius ;
            %只需要记录距离障碍物最近的点，所以不考虑最远距离
        else                                                             %如果障碍物圆心在检测角度外，返回最短距离=√dist_real^2-d_line^2-√r^2-d_line^2
            square_d1 = (a-Qx(1))^2;
            square_d2 = square_d1;
            if(kk1_value);square_d1 = dist_pointToline(kk1,b1,centre);   end   %计算圆心到扇形两个边界距离的平方
            if(kk2_value);square_d2 = dist_pointToline(kk2,b2,centre);   end
            if(square_d1 <= radius^2 && square_d1 <= square_d2)    %如果 扇形边长与圆相交或者相切
                dist1 = sqrt(dist_real^2-square_d1)-sqrt(radius^2-square_d1);
                if(dist1<sensor_value); sensor_value = dist1 ;end
            elseif(square_d2 < radius^2 && square_d1 > square_d2)
                dist2 = sqrt(dist_real^2-square_d2)-sqrt(radius^2-square_d2);
                if(dist2<sensor_value); sensor_value = dist2 ;end
            end
        end
    end
    %检测碰撞,这个碰撞是用来学习使用的，所以考虑传感器探测距离而不是小车传感器与障碍物边界点的实际距离
    
    
    
    %  检测固态障碍物
elseif( strcmp(shape,'polygon') )
    
    n_punkter = size(P,1);     %多边形顶点数量
    
    
    for i = 1:n_punkter-1   % 这里计算的是n-1个边
        
        
        b = P(i+1, 2) - P(i, 2);
        c = P(i+1, 1) - P(i, 1);
        middle_x=( P(i+1, 1) + P(i, 1))/2;
        middle_y=( P(i+1, 2) + P(i, 2))/2;        
        theta_pi = ang_sensorToVertical( sensor_vector,[(P(i, 1) - Qx(1)),(P(i, 2) - Qy(1))]);    % 传感器到端点i向量与中轴线夹角
        theta_pj = ang_sensorToVertical( sensor_vector,[(P(i+1, 1) - Qx(1)),(P(i+1, 2) - Qy(1))]);  % 传感器到端点i+1向量与中轴线夹角
        theta_middle = ang_sensorToVertical( sensor_vector,[(middle_x - Qx(1)),(middle_y - Qy(1))]);  % 传感器到端点i+1向量与中轴线夹角
        if(theta_pi<theta_pj)     % 这个用来把两个端点用到传感器的角度大小进行排序，并且保留vertice对应的这个两个端点的坐标便于索引
            theta_p1 = theta_pi;
            theta_p2 = theta_pj;
            vertice1=[P(i,1),P(i,2)];
            vertice2=[P(i+1,1),P(i+1,2)];
        else
            theta_p1 = theta_pj;
            theta_p2 = theta_pi;
            vertice2=[P(i,1),P(i,2)];
            vertice1=[P(i+1,1),P(i+1,2)];
        end
        if( abs(c) <= 0.001 );kkp_value = false; else; kkp_value=true;  kk_p  = b/c; b_p = -kk_p * P(i, 1) + P(i, 2);       end
        if( abs(b) <= 0.001 );kkc_value = false; else; kkc_value=true;  kk_c=-c/b;  b_c = -kk_c * Qx(1) + Qy(1);            end
        if(~kkc_value)
            dist_patch = (Qy(1)-P(i,2))^2;     %两个纵坐标差的平方和
        elseif(~kkp_value)
            dist_patch = (Qx(1)-P(i, 1))^2;    %两个横坐标差的平方和
        else
            dist_patch = dist_pointToline(kk_p,b_p,Q_coord);
            
        end

        if((~(theta_p2<-theta1) && ~(theta_p1>theta1))&&((theta_p1<theta_middle)&&(theta_p2>theta_middle)))
            theta_vector = [max([-theta1,theta_p1]),min([theta1,theta_p2])];   % 得到检测范围和 边界范围的交集
            
            if(theta_vector(1)==theta_p1)% 如果交集左端点是 边界
                coord_1 =  vertice1;
                
            else                  %     if(theta_vector(1)==theta_1)
                if(~kk1_value && ~kkp_value)
                    coord_1 =  vertice1;
                elseif(~kk1_value && kkp_value)
                    x_vertical = Qx(1);
                    y_vertical = kk_p * x_vertical+b_p;
                    coord_1 = [x_vertical,y_vertical];
                elseif(~kkp_value && kk1_value)
                    x_vertical = P(i,1);
                    y_vertical = kk1 * x_vertical+b1;
                    coord_1 = [x_vertical,y_vertical];
                else
                    x_vertical = (b1-b_p)/(kk_p-kk1);  %更新新的障碍物边长坐标
                    y_vertical = kk1*x_vertical+b1;
                    coord_1 = [x_vertical,y_vertical];
                end
                
            end
            if((theta_vector(2)==theta_p2))
                coord_2 = vertice2;
                
            else                   %     if(theta_vector(2)==theta_2)
                if(~kk2_value && ~kkp_value)
                    coord_2 = vertice2;
                elseif(~kk2_value)
                    x_vertical = Qx(1);
                    y_vertical = kk_p * x_vertical+b_p;
                    coord_2 = [x_vertical,y_vertical];
                elseif(~kkp_value)
                    x_vertical = P(i,1);
                    y_vertical = kk2 * x_vertical+b2;
                    coord_2 = [x_vertical,y_vertical];
                else
                    x_vertical = (b2-b_p)/(kk_p-kk2);  %更新新的障碍物边长坐标
                    y_vertical = kk2*x_vertical+b2;
                    coord_2 = [x_vertical,y_vertical];
                end
                
            end
            x_limit1 = min([coord_1(1), coord_2(1)]);     % x-min
            x_limit2 = max([coord_1(1), coord_2(1)]);     % x-max
            y_limit1 = min([coord_1(2), coord_2(2)]);     % y-min
            y_limit2 = max([coord_1(2), coord_2(2)]);     % y-max
            if(~kkc_value)
                x_vertical = Qx(1);
                if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %如果垂足在边长范围内，返回垂线段长度，否则返回到达最近顶点的距离
                    sensor_temp = sqrt(dist_patch);
                else
                    sensor_temp = min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
                end
            elseif(~kkp_value)
                y_vertical = Qy(1);    %  如果多边形边长垂直x轴， 这里计算垂足纵坐标
                if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
                    sensor_temp = sqrt(dist_patch);
                else
                    sensor_temp = min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
                end
            else
                x_vertical = (b_c-b_p)/(kk_p-kk_c);
                if(x_vertical>=x_limit1 && x_vertical<=x_limit2)
                    sensor_temp = sqrt(dist_patch);
                else
                    sensor_temp = min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
                end
            end

            if(sensor_value>sensor_temp); sensor_value=sensor_temp;end    % 保留最短的检测值
        end
        
    end
    
    
    
    %对于多边形障碍物最后一个边
    
    

    b = P(n_punkter, 2) - P(1, 2);
    c = P(n_punkter, 1) - P(1, 1);
    middle_x=( P(n_punkter, 1) + P(1, 1))/2;
    middle_y=( P(n_punkter, 2) + P(1, 2))/2;
    theta_pi = ang_sensorToVertical( sensor_vector,[(P(1, 1) - Qx(1)),(P(1, 2) - Qy(1))]);    % 传感器到端点i向量与中轴线夹角
    theta_pj = ang_sensorToVertical( sensor_vector,[(P(n_punkter, 1) - Qx(1)),(P(n_punkter, 2) - Qy(1))]);  % 传感器到端点i+1向量与中轴线夹角
    theta_middle = ang_sensorToVertical( sensor_vector,[(middle_x - Qx(1)),(middle_y - Qy(1))]);  % 传感器到端点i+1向量与中轴线夹角
    if(theta_pi<theta_pj)     % 这个用来把两个端点用到传感器的角度大小进行排序，并且保留vertice对应的这个两个端点的坐标便于索引
        theta_p1 = theta_pi;
        theta_p2 = theta_pj;
        vertice1=[P(1,1),P(1,2)];
        vertice2=[P(n_punkter,1),P(n_punkter,2)];
    else
        theta_p1 = theta_pj;
        theta_p2 = theta_pi;
        vertice2=[P(1,1),P(1,2)];
        vertice1=[P(n_punkter,1),P(n_punkter,2)];
    end
    if( abs(c) <= 0.001 );kkp_value = false; else; kkp_value=true;  kk_p  = b/c;  kk_c = 0;     b_p = -kk_p * P(1, 1) + P(1, 2);          end
    if( abs(b) <= 0.001 );kkc_value = false; else; kkc_value=true;  kk_c=-c/b;     b_c = -kk_c * Qx(1) + Qy(1);          end

    
    if(~kkc_value)
        dist_patch = (Qy(1)-P(1,2))^2;
    elseif(~kkp_value)
        dist_patch = (Qx(1)-P(1, 1))^2;
    else
        dist_patch = dist_pointToline(kk_p,b_p,Q_coord);
    end

    
    if( ~(theta_p2<-theta1) && ~(theta_p1>theta1)&&((theta_p1<theta_middle)&&(theta_p2>theta_middle)))
        theta_vector = [max([-theta1,theta_p1]),min([theta1,theta_p2])];
        
        if(theta_vector(1)==theta_p1)% 如果交集左端点是 边界或者
            coord_1 =  vertice1;
            
        else                  %     if(theta_vector(1)==theta_1)
            if(~kk1_value && ~kkp_value)
                coord_1 =  vertice1;
            elseif(~kk1_value && kkp_value)
                x_vertical = Qx(1);
                y_vertical = kk_p * x_vertical+b_p;
                coord_1 = [x_vertical,y_vertical];
            elseif(~kkp_value && kk1_value)
                x_vertical = P(1,1);
                y_vertical = kk1 * x_vertical+b1;
                coord_1 = [x_vertical,y_vertical];
            else
                x_vertical = (b1-b_p)/(kk_p-kk1);  %更新新的障碍物边长坐标
                y_vertical = kk1*x_vertical+b1;
                coord_1 = [x_vertical,y_vertical];
            end
            
        end
        if((theta_vector(2)==theta_p2))
            coord_2 = vertice2;
            
        else                   %     if(theta_vector(2)==theta_2)
            if(~kk2_value && ~kkp_value)
                coord_2 = vertice2;
            elseif(~kk2_value)
                x_vertical = Qx(1);
                y_vertical = kk_p * x_vertical+b_p;
                coord_2 = [x_vertical,y_vertical];
            elseif(~kkp_value)
                x_vertical = P(1,1);
                y_vertical = kk2 * x_vertical+b2;
                coord_2 = [x_vertical,y_vertical];
            else
                x_vertical = (b2-b_p)/(kk_p-kk2);  %更新新的障碍物边长坐标
                y_vertical = kk2*x_vertical+b2;
                coord_2 = [x_vertical,y_vertical];
            end
            
        end
        x_limit1 = min([coord_1(1), coord_2(1)]);     % x-min
        x_limit2 = max([coord_1(1), coord_2(1)]);     % x-max
        y_limit1 = min([coord_1(2), coord_2(2)]);     % x-min
        y_limit2 = max([coord_1(2), coord_2(2)]);     % x-max
        if(~kkc_value)
            x_vertical = Qx(1);
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %如果垂足在边长范围内，返回垂线段长度，否则返回到达最近顶点的距离
                sensor_temp = sqrt(dist_patch);
            else
                sensor_temp =min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
            end
        elseif(~kkp_value)
            y_vertical = Qy(1);    %  如果多边形边长垂直x轴， 这里计算垂足纵坐标
            if(y_vertical>=y_limit1 && y_vertical<=y_limit2)
                sensor_temp = sqrt(dist_patch);
            else
                sensor_temp =min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
            end
        else
            x_vertical = (b_c-b_p)/(kk_p-kk_c);
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)
                sensor_temp = sqrt(dist_patch);
            else
                sensor_temp = min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
            end
        end

        if(sensor_value>sensor_temp); sensor_value=sensor_temp;end    % 保留最短的检测值
        
    end
    
    
    
    
end
if(sensor_value < dist_crash)
    crash = true;
end
end
% end
%改进思路

% 1.改y值加减为欧氏距离 直接算圆心距，使用平方做比较不要开方来减少计算量
% 2.计算到多边形的距离时，计算传感器起始点到线段的垂直距离，如果小于探测距离，判断距离传感器最近的点是否在探测区域，然后返回一个距离值


% 逻辑关系
%1. 如果垂直距离大于100 则不予判定
%       如果两个端点与传感器点的扇形角度（利用中轴线计算） 和 扇形检测区域 -30，30没有交集，则不予判定
%以上合并到一个语句

%求得最新的角度区间
%  如果是端点角度 直接返回端点距离值
%  如果是传感器扇形角度，先计算交点坐标 ，然后计算传感器与交点距离值
%    ang=ang_sensorToVertical( sensor_vector,vertical_vector);
% 把 ==0 换成 <=0.001
%如果 存在端点不在传感器范围内   更新新的边长点
%先判断距离，然后再更新新的坐标点，然后再判断距离或者垂直距离
%坐标更新完了，还差把垂直距离加进来