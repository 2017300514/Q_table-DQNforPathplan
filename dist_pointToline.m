function square_dist = dist_pointToline(k,b,point_pisition)
    %���������������㵽ֱ�ߵľ���ƽ��
    %k,b ֱ�ߵ�б�ʺͽؾ�
    x = point_pisition(1);
    y = point_pisition(2);
    square_dist = (k*x-y+b)^2/(k^2+1);

end