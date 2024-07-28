function [sensor, crash] = checkCrash(car, obstacle_polygon, obstacle_circle)
    %% ����˵��  ���ײ������
    %{
    ���� ������ģ��(��������ʹ�����) ������ϰ����Բ�ϰ���ģ��
    ���  ��������ֵ  ����ײ�������
    %}
    
    %% ----- Initialization  ��ʼ������

     A = get(car(1),'Vertices');     %Get vertices of the vehicle     ��ȡ������������ֵ
%     dist_crash = [1,1,1,1,1,1,1,1,1,1,0.3,0.3,0.3,0.3];               %Sensor distance (1.5 = 15cm) for crash    ������̽��ײ������  15cm
    dist_crash = [1,1,1,1,1,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    obs_crash = false;              %Obstacle crash     �ϰ���ײ��
    sens_crash = false;             %Crash sensor values are below 1.5.  ̽��������1.5dm
    status = true;                  %Help variable       ��������

    %% ----- Check collision between the vehicle and obstacles
    
    n_polygons = length(obstacle_polygon);              %���������ϰ��������
    n_circles = length(obstacle_circle);                %����Բ���ϰ��������
    for i = 1:n_polygons

        B = get(obstacle_polygon(i),'Vertices');        %��ö���ε����ж�������ֵ 

        obs_crash = obstacleCrash1(A, B, 'polygon');     %�ж��Ƿ���ײ

        if(obs_crash)
            status = false; 
            break;                  
        end

    end

    if(status)    %�ж��Ƿ���ײԲ���ϰ���

        for i = 1:n_circles

            C = get(obstacle_circle(i),'Vertices');

            obs_crash = obstacleCrash1(A, C, 'circle');

            if(obs_crash)
                break;
            end
        end
    end

    %% ----- Check collision with sensors
    
    n_sensors = length(car(2:end));   %���崫����������
    sensor = zeros(1, n_sensors);     % ��ʼ����������ֵ
    
    % ����������
    for i = 1:n_sensors
        
           
        D = get(car(i+1),'Vertices');   %  ��ȡ����������
        sensor_temp = 0;
        
        % ����ÿ�����������ж�����Բ���ϰ���
        for j = 1:n_circles
            
            C = get(obstacle_circle(j), 'Vertices');
            
            % ��һ�������ﴢ�����еĴ�������ֵ
            [sensor_temp(j), crash] = sensorValues1(D, C, 'circle', dist_crash(i));
            
            % �����ײ
            if(crash)    %�޸Ĺ���Ŀ����Ϊ�˼�����ײ���ж���Χ
                sens_crash = true;
            end
        end


        for k = (j+1):(j + n_polygons)   % ������Բ���ϰ�������ֵ�ص������Դ�j+1��ʼ��¼

            B = get(obstacle_polygon(k-j), 'Vertices');

            [sensor_temp(k), crash] = sensorValues1(D, B, 'polygon', dist_crash(i));

            if(crash)   

                sens_crash = true;
            end
        end
        
        % ��¼��̵Ĵ�������ֵ
        sensor(i) = min(sensor_temp);

    end

    %sensor = round(10.*sensor);
    sensor = 10.*sensor;                % ���Ա��� 1:10
    disp(sensor);
    crash = [obs_crash, sens_crash];    % �������е���ײ���
end