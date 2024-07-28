function value = obstacle_angle( sensor,ob_punkt,sensor_punkt)  
    % ������������С�����������ϰ����н�
    %  sensor  ����������������
    %  ob_punkt �ϰ����
    value = false;                                   % ��ʼ���ж�Ŀ��㲻�ٴ�������Χ��
    theta=30/180*pi;
%     A = get(sensor,'Vertices');                      % ��ȡ������ģ�Ͷ˵㣬
    w1 = sensor';              % �õ�������������������߷���
    w2 = [ob_punkt(1)-sensor_punkt(1);ob_punkt(2)-sensor_punkt(2)];    % С��Բ�ĵ�Ŀ���ľ�������
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));          % ����arcos �õ��Ƕȵľ���ֵ���ж��������ڼ���
    if (ang <= theta)
        value = true;
    end
end
    