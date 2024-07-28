function [x, y] = circle1(senter, radius, angle, n_points)      
    
	%  画扇形函数 利用极坐标转笛卡尔坐标系

    s = senter;
    r = radius;
    n = n_points;
    
    theta = linspace(angle(1),angle(2), n);
    rho = ones(1, n)*r;
    [x, y] = pol2cart(theta, rho);   
    x = x + s(1);
    y = y + s(2);
    x = [x, s(1)];
    y = [y, s(2)];
    
    
end