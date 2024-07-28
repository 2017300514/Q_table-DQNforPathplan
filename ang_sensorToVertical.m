function ang = ang_sensorToVertical( sensor_vector,vertical_vector)  
    % 函数用来计算两个向量夹角，并以w1左侧为正，以w1右侧为负

    
    w1 = sensor_vector';  %得到小车运动方向
    w2 = vertical_vector';    %小车圆心到目标点的距离向量
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));  % 计算arcos 得到角度
    m=cross([w1;0],[w2;0]);
    if(m(3)<0)
        ang = -ang;     %定义目标在中轴线左侧角度为正
    end   
end
    