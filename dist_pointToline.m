function square_dist = dist_pointToline(k,b,point_pisition)
    %这个函数用来计算点到直线的距离平方
    %k,b 直线的斜率和截距
    x = point_pisition(1);
    y = point_pisition(2);
    square_dist = (k*x-y+b)^2/(k^2+1);

end