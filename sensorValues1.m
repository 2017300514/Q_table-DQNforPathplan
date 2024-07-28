function [sensor_value, crash] = sensorValues1(sensor_vertices, obstacle, shape, dist_crash)

%���㴫�������Ŀ���ľ���
%sensor_vertices  ������patch
%obstacle   �ϰ��� patch
%shape   �ϰ�������

%dist_crash  ��������ײ��ⷶΧ



crash = false;

% k_value=true;               % ��ʼ������ֱ�ߵ�б��״̬��������б��
kk1_value=true;
kk2_value=true;

%     dist1=0;                     %�����������̾���dist1,dist2
%     dist2=0;
theta1=pi/6;
theta2=2*pi-theta1;
kk2=0;                      %����kk2�Ƕ�Ӧ��ʱ����ת��ʮ�ȵ�ֱ�ߣ����̵�б��
kk1=0;                      %����kk1�Ƕ�Ӧ˳ʱ����ת��ʮ�ȵ�ֱ�ߣ����̵�б��
P = obstacle;                   %   �ϰ���˵�����
Qx = sensor_vertices(:,1);      %    ������ģ�Ͷ˵��x����
Qy = sensor_vertices(:,2);      %    ������ģ�Ͷ˵��y����
sensor_vector= [Qx(end) - Qx(1), Qy(end) - Qy(1)];
Q_coord=[Qx(1),Qy(1)];
sensor_length = norm(sensor_vector);    % ������̽��������
sensor_value = sensor_length;           % Ĭ��û�м�⵽�ϰ��ﷵ��������
%     ���㴫�����ķ��̣�ֱ�ߣ�
b = Qy(end)- Qy(1);
c = Qx(end)- Qx(1);    %�������㴫����ֱ�߷���
if( abs(c) > 0.001 )
    k=b/c;
    if( abs(k-1.732)<= 0.001 )    % �������������ʮ�ȸ�������ʱ����ת30��û��б��
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
        b1 = Qy(1)-kk1*Qx(1);         % ����˳ʱ��߽���2 ֱ�߷���  y=kk2*x+b2 ��бʽ
        b2 = Qy(1)-kk2*Qx(1);         % ������ʱ��߽���1 ֱ�߷���  y=kk1*x+b1 ��бʽ
    end
else                                  % ��������������ߴ�ֱx�ᣬk�����ڣ�ֱ�ӷ���������תб��
    kk2=-1.732;                       %  kk2����ʱ���б��
    kk1=+1.732;                       %  kk1����ʱ���б��
    b2 = Qy(1)-kk2*Qx(1);
    b1 = Qy(1)-kk1*Qx(1);
end

%���Բ���ϰ���
if( strcmp(shape,'circle') )
    
    n_punkter = size(P,1);
    a = P(n_punkter,1);
    b = P(n_punkter,2);
    centre = [a, b];
    radius = norm([P(1,1) - a, P(1,2) - b] );   %�����ϰ���İ뾶
    dist_real=pdist([centre;Q_coord],'euclidean');                          %�ϰ����봫����Բ�ľ�
    deta_theta=asin(radius/dist_real);
    help_1=sensor_vector'; 
    dist_help=(centre-Q_coord)';
    help_2 =cross([help_1;0],[dist_help;0]);                                      
    
    theta = acos(dist_help'*help_1/(norm(dist_help)*norm(help_1)));  % ����Բ�������봫���������ļн�
    if (help_2(3)<0)
        theta = -theta;
    end
   
    lamda2=theta+deta_theta;
    lamda1=theta-deta_theta;   % lamda�ǿ������α߽�������һ������������αߵļн�
                                  
    if(dist_real<=sensor_length+radius&&(~(lamda1>pi/6||lamda2<-pi/6)))                               %����ϰ��������ܼ�ⷶΧ, d-r<̽�����
        if(obstacle_angle( sensor_vector,centre,Q_coord) )   %����ϰ���Բ���ڼ��Ƕ��ڣ�������̾���=d-r
            sensor_value = dist_real-radius ;
            %ֻ��Ҫ��¼�����ϰ�������ĵ㣬���Բ�������Զ����
        else                                                             %����ϰ���Բ���ڼ��Ƕ��⣬������̾���=��dist_real^2-d_line^2-��r^2-d_line^2
            square_d1 = (a-Qx(1))^2;
            square_d2 = square_d1;
            if(kk1_value);square_d1 = dist_pointToline(kk1,b1,centre);   end   %����Բ�ĵ����������߽�����ƽ��
            if(kk2_value);square_d2 = dist_pointToline(kk2,b2,centre);   end
            if(square_d1 <= radius^2 && square_d1 <= square_d2)    %��� ���α߳���Բ�ཻ��������
                dist1 = sqrt(dist_real^2-square_d1)-sqrt(radius^2-square_d1);
                if(dist1<sensor_value); sensor_value = dist1 ;end
            elseif(square_d2 < radius^2 && square_d1 > square_d2)
                dist2 = sqrt(dist_real^2-square_d2)-sqrt(radius^2-square_d2);
                if(dist2<sensor_value); sensor_value = dist2 ;end
            end
        end
    end
    %�����ײ,�����ײ������ѧϰʹ�õģ����Կ��Ǵ�����̽����������С�����������ϰ���߽���ʵ�ʾ���
    
    
    
    %  ����̬�ϰ���
elseif( strcmp(shape,'polygon') )
    
    n_punkter = size(P,1);     %����ζ�������
    
    
    for i = 1:n_punkter-1   % ����������n-1����
        
        
        b = P(i+1, 2) - P(i, 2);
        c = P(i+1, 1) - P(i, 1);
        middle_x=( P(i+1, 1) + P(i, 1))/2;
        middle_y=( P(i+1, 2) + P(i, 2))/2;        
        theta_pi = ang_sensorToVertical( sensor_vector,[(P(i, 1) - Qx(1)),(P(i, 2) - Qy(1))]);    % ���������˵�i�����������߼н�
        theta_pj = ang_sensorToVertical( sensor_vector,[(P(i+1, 1) - Qx(1)),(P(i+1, 2) - Qy(1))]);  % ���������˵�i+1�����������߼н�
        theta_middle = ang_sensorToVertical( sensor_vector,[(middle_x - Qx(1)),(middle_y - Qy(1))]);  % ���������˵�i+1�����������߼н�
        if(theta_pi<theta_pj)     % ��������������˵��õ��������ĽǶȴ�С�������򣬲��ұ���vertice��Ӧ����������˵�������������
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
            dist_patch = (Qy(1)-P(i,2))^2;     %������������ƽ����
        elseif(~kkp_value)
            dist_patch = (Qx(1)-P(i, 1))^2;    %������������ƽ����
        else
            dist_patch = dist_pointToline(kk_p,b_p,Q_coord);
            
        end

        if((~(theta_p2<-theta1) && ~(theta_p1>theta1))&&((theta_p1<theta_middle)&&(theta_p2>theta_middle)))
            theta_vector = [max([-theta1,theta_p1]),min([theta1,theta_p2])];   % �õ���ⷶΧ�� �߽緶Χ�Ľ���
            
            if(theta_vector(1)==theta_p1)% ���������˵��� �߽�
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
                    x_vertical = (b1-b_p)/(kk_p-kk1);  %�����µ��ϰ���߳�����
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
                    x_vertical = (b2-b_p)/(kk_p-kk2);  %�����µ��ϰ���߳�����
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
                if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %��������ڱ߳���Χ�ڣ����ش��߶γ��ȣ����򷵻ص����������ľ���
                    sensor_temp = sqrt(dist_patch);
                else
                    sensor_temp = min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
                end
            elseif(~kkp_value)
                y_vertical = Qy(1);    %  �������α߳���ֱx�ᣬ ������㴹��������
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

            if(sensor_value>sensor_temp); sensor_value=sensor_temp;end    % ������̵ļ��ֵ
        end
        
    end
    
    
    
    %���ڶ�����ϰ������һ����
    
    

    b = P(n_punkter, 2) - P(1, 2);
    c = P(n_punkter, 1) - P(1, 1);
    middle_x=( P(n_punkter, 1) + P(1, 1))/2;
    middle_y=( P(n_punkter, 2) + P(1, 2))/2;
    theta_pi = ang_sensorToVertical( sensor_vector,[(P(1, 1) - Qx(1)),(P(1, 2) - Qy(1))]);    % ���������˵�i�����������߼н�
    theta_pj = ang_sensorToVertical( sensor_vector,[(P(n_punkter, 1) - Qx(1)),(P(n_punkter, 2) - Qy(1))]);  % ���������˵�i+1�����������߼н�
    theta_middle = ang_sensorToVertical( sensor_vector,[(middle_x - Qx(1)),(middle_y - Qy(1))]);  % ���������˵�i+1�����������߼н�
    if(theta_pi<theta_pj)     % ��������������˵��õ��������ĽǶȴ�С�������򣬲��ұ���vertice��Ӧ����������˵�������������
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
        
        if(theta_vector(1)==theta_p1)% ���������˵��� �߽����
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
                x_vertical = (b1-b_p)/(kk_p-kk1);  %�����µ��ϰ���߳�����
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
                x_vertical = (b2-b_p)/(kk_p-kk2);  %�����µ��ϰ���߳�����
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
            if(x_vertical>=x_limit1 && x_vertical<=x_limit2)    %��������ڱ߳���Χ�ڣ����ش��߶γ��ȣ����򷵻ص����������ľ���
                sensor_temp = sqrt(dist_patch);
            else
                sensor_temp =min([pdist([Q_coord;coord_1],'euclidean'),pdist([Q_coord;coord_2],'euclidean')]);
            end
        elseif(~kkp_value)
            y_vertical = Qy(1);    %  �������α߳���ֱx�ᣬ ������㴹��������
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

        if(sensor_value>sensor_temp); sensor_value=sensor_temp;end    % ������̵ļ��ֵ
        
    end
    
    
    
    
end
if(sensor_value < dist_crash)
    crash = true;
end
end
% end
%�Ľ�˼·

% 1.��yֵ�Ӽ�Ϊŷ�Ͼ��� ֱ����Բ�ľ࣬ʹ��ƽ�����Ƚϲ�Ҫ���������ټ�����
% 2.���㵽����εľ���ʱ�����㴫������ʼ�㵽�߶εĴ�ֱ���룬���С��̽����룬�жϾ��봫��������ĵ��Ƿ���̽������Ȼ�󷵻�һ������ֵ


% �߼���ϵ
%1. �����ֱ�������100 �����ж�
%       ��������˵��봫����������νǶȣ����������߼��㣩 �� ���μ������ -30��30û�н����������ж�
%���Ϻϲ���һ�����

%������µĽǶ�����
%  ����Ƕ˵�Ƕ� ֱ�ӷ��ض˵����ֵ
%  ����Ǵ��������νǶȣ��ȼ��㽻������ ��Ȼ����㴫�����뽻�����ֵ
%    ang=ang_sensorToVertical( sensor_vector,vertical_vector);
% �� ==0 ���� <=0.001
%��� ���ڶ˵㲻�ڴ�������Χ��   �����µı߳���
%���жϾ��룬Ȼ���ٸ����µ�����㣬Ȼ�����жϾ�����ߴ�ֱ����
%����������ˣ�����Ѵ�ֱ����ӽ���