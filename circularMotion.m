function circularMotion(point, patch, theta)

    %初始化小车时，整体旋转小车
    
    for i = 1:length(patch)
        
        rotate(patch(i), [0,0,1], theta*180/pi, [point,0]);     
    end
    
end