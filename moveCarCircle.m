function rot_punkt = moveCarCircle(velocity, tot_rot, theta, patch)  %速度 
    
    %% 程序说明 移动小车车体的函数
    
    P = get(patch,'Vertices');
    r = ones(size(P));

    r = P(:,1:2) + velocity.*[cos(tot_rot)*r(:,1), sin(tot_rot)*r(:,2)];
    set(patch, 'XData', r(:,1), 'YData', r(:,2));

    center_x  = (min(r(:,1)) + max(r(:,1)))/2;
    center_y = (min(r(:,2)) + max(r(:,2)))/2;
    rot_punkt = [center_x, center_y];                  % center of the vehicle
    rotate(patch,[0,0,1], (theta*180)/pi, [rot_punkt,0]);
end
