function [sensor_value, crash] = sensorValues(sensor_vertices, obstacle, shape, dist_crash)

    %���㴫�������Ŀ���ľ���
    %sensor_vertices  ������patch �ϵ�λ����Ϣ
    

    crash = false;

    P = obstacle;                   %   �ϰ���˵�����
    Qx = sensor_vertices(:,1);      %    ���������x����   
    Qy = sensor_vertices(:,2);      %    ���������y����
    sensor_value = norm([Qx(end) - Qx(1), Qy(end) - Qy(1)]);    % �������ĳ��ȣ�Ĭ��û�м�⵽�ϰ��ﷵ����󳤶�

    Qx_min = min(Qx);   %������ģ�͵����x             
    Qx_max = max(Qx);   %������ģ�͵����x

    Qy_min = min(Qy);  %������ģ�͵����y
    Qy_max = max(Qy);  %������ģ�͵��յ�y

    n_points = 7000;
    x = linspace(Qx_min, Qx_max, n_points)';    %�߶��Ͼ���7000����x����
    y = linspace(Qy_min, Qy_max, n_points)';    %�߶��Ͼ���7000����y����
    n_x = length(x);        %    ���������ϵ������
    dist_between = 0.2;     %    ���������ϰ���֮���������С���� ���������ײ

    %     ���㴫�����ķ��̣�ֱ�ߣ�
    b = Qy(end)- Qy(1);
    c = Qx(end)- Qx(1);

    if( c ~= 0 )
        a = b/c;                        %   ����б��
        y = a.*(x - Qx(1)) + Qy(1);     %   ֱ�߷���  
    else
        n_points = length(y);
        x = Qx(1).*ones(n_points,1);
    end

    %���Բ���ϰ���
    if( strcmp(shape,'circle') )  
        
        a = (min(P(:,1)) + max(P(:,1)))/2;
        b = (min(P(:,2)) + max(P(:,2)))/2;

        centre = [a, b];
        radius = norm([P(1,1) - centre(1), P(1,2) - centre(2)] );   %�����ϰ���İ뾶


        f1 =  sqrt( radius.^2 - (x - a).^2 ) + b;
        f2 = -sqrt( radius.^2 - (x - a).^2 ) + b;
        
        %Sjekker differansen mellom sensorens funksjon y og hindringens funksjon f
        g1 = abs(y - f1);
        g2 = abs(y - f2);
        
        %�����ײ����¼�������
        I1 = find( g1 < dist_between);
        I2 = find( g2 < dist_between);
        I = [I1; I2];
          
        if( ~isempty(I) )
            
            %    ���ؾ�������ĵ���Ϊ�������ľ�����ֵ   ������һ��min����
            sensor_value = min(dist([x(I), y(I)], [Qx(1); Qy(1)]));
            
            %�����ײ
            if(sensor_value < dist_crash)

                crash = true;
            end

        end

    %  ����̬�ϰ���
    elseif( strcmp(shape,'polygon') )

        n_punkter = size(P,1);
        
        %f(x) = a*(x-x0) + f(x0) for kantene av hindringen
        for i = 1:n_punkter-1   % ����������n-1����
  
            f = ones(n_x,1);
            g = ones(n_x,1);
            b = P(i+1, 2) - P(i, 2);
            c = P(i+1, 1) - P(i, 1);

            if( c ~= 0 )

                a = b/c;

                x_limit1 = min([P(i,1), P(i+1,1)]);     %x-min grense
                x_limit2 = max([P(i,1), P(i+1,1)]);     %x-maks grense

                %���� �����������ϵĵ��� �߳��˵�x���귶Χ��
                I = (x_limit1 < x) & (x < x_limit2);    

                f(I) = a.*(x(I) - P(i,1)) + P(i,2);     %funksjonen f(x)
                g(I) = abs(y(I) - f(I));                %diffensen mellom y og f(x)

            else   %  �������α߳��Ǵ�ֱ�ģ����غ������ֵ��Ϊ����

                y_limit1 = min([P(i,2), P(i+1,2)]);     %y-min grense
                y_limit2 = max([P(i,2), P(i+1,2)]);     %y-maks grense
            
                %   ���� �����������ϵĵ��� �߳��˵�y���귶Χ��
                I = (y_limit1 < y) & (y < y_limit2);
                
                
                g(I) = abs( x(I) - P(i,1) );            

            end
            
            %��¼��̵Ĵ�������ֵ����Ϊ������ж���ߣ�Ҫ����Σ��ȽϺ�����Сֵ
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
        
        
        %���ڶ�����ϰ������һ����
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

%�Ľ�˼· 

% 1.��yֵ�Ӽ�Ϊŷ�Ͼ��� ֱ����Բ�ľ࣬ʹ��ƽ�����Ƚϲ�Ҫ���������ټ�����
% 2.���㵽����εľ���ʱ�����㴫������ʼ�㵽�߶εĴ�ֱ���룬���С��̽����룬�жϾ��봫��������ĵ��Ƿ���̽������Ȼ�󷵻�һ������ֵ