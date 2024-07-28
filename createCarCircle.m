function h = createCarCircle(radius, sensor_lengde, color,init_plot, theta)

    %% ��������ĺ���
    %{
        ���룺 radius  ������뾶
        sensor_lengde  �����崫����ģ�ͷ�Χ
        color          ����������ģ����ɫ
        init_plot      ������λ��
        theta          �������ʼ��׼��λ �ɼ�����0�ȿ�ʼ����ʱ��Ϊ����˳ʱ��Ϊ����
        ����� h       ������patch����
                 
    %}
    x_set = init_plot(1);
    y_set = init_plot(2);

    
    [X,Y] = circle(init_plot, radius, 100); 
    car = patch(X,Y, color);      % ���                              
    set(car,'FaceLighting','phong','EdgeLighting','phong');

    
    n_punkter = 2;              % �����������ϵĵ�

    %x-aksen for de 5 forskjellige sensor-linjene   �����ͬ��������x��
    x1 = linspace(radius, sensor_lengde + (radius), n_punkter)+x_set;             %middle sensor. 0 grader       
    x2 = linspace(radius, sensor_lengde*cos(pi/4) + radius, n_punkter)+x_set;     %the two sensors close to middle. pi/4 radianer
    x3 = linspace(radius, sensor_lengde*cos(pi/8) + radius, n_punkter)+x_set;     %the two farthest sensors. pi/8 radianer  ����
    x4 = (-1)*linspace(radius, sensor_lengde + (radius), n_punkter)+x_set;
    x5 = (-1)*linspace(radius, sensor_lengde*cos(pi/4) + radius, n_punkter)+x_set;
    x6 = (-1)*linspace(radius, sensor_lengde*cos(pi/8) + radius, n_punkter)+x_set;
    y7 =  linspace(radius*sin(pi/4), sensor_lengde+radius*sin(pi/4), n_punkter)+y_set;
    y8 =  (-1)*linspace(radius*sin(pi/4), sensor_lengde+radius*sin(pi/4), n_punkter)+y_set;
    for i = 1:n_punkter
        
        % Equations for straight lines (y-aksen)
        y1(i) = tan(pi/4)*(x2(i) -radius-x_set)+y_set;      
        y2(i) = tan(pi/8)*(-radius + x3(i)-x_set)+y_set;
        y3(i) = 0+y_set; 
        y4(i) = tan(pi/8)*(radius - x3(i)+x_set)+y_set;
        y5(i) = tan(pi/4)*(radius - x2(i)+x_set)+y_set;
        y6(i) = tan(pi/8)*(+radius + x6(i)-x_set)+y_set;
        x7(i) = cos(pi/4)*radius+ x_set;
        x8(i) = (-1)*cos(pi/4)*radius+x_set;
    end
    % Create sensors
    % ǰ�����������
    L_h1 = patch(x2', y1', 'k');
    L_h2 = patch(x3', y2', 'k');
    L_h3 = patch(x1', y3', 'k');
    L_h4 = patch(x3', y4', 'k');
    L_h5 = patch(x2', y5', 'k');  %��ɫ
    % �����������
    L_h6 = patch(x5', y1', 'k');
    L_h7 = patch(x6', y2', 'k');    
    L_h8 = patch(x4', y3', 'k');    
    L_h9 = patch(x6', y4', 'k');
    L_h10 = patch(x5', y5', 'k');
    % ���Ҳ��ĸ�������
    L_h11 = patch(x7', y7', 'k');
    L_h12 = patch(x8', y7', 'k');
    L_h13 = patch(x7', y8', 'k');
    L_h14 = patch(x8', y8', 'k');
    h = [car ,L_h1, L_h2, L_h3, L_h4, L_h5, L_h6, L_h7, L_h8, L_h9, L_h10, L_h11, L_h12, L_h13, L_h14];   
%     h = [car ,L_h1, L_h2, L_h3, L_h4, L_h5, L_h9]; 
    circularMotion(init_plot,h,theta);
end