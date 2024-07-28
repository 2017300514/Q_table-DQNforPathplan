function ang = table_cos_to_angle( car,rot_punkt,goal)  
    % 函数用来计算小车与目标视线角
    %  car  小车与传感器整体 patch对象
    %  rot_punkt 小车车体圆心
    %  goal 目标点位置
    A = get(car(4),'Vertices');   % 获取小车中轴线处传感器模型端点，
    w1 = [A(2,1)-A(1,1);A(2,2)-A(1,2)];  %得到小车运动方向
    w2 = [goal(1)-rot_punkt(1);goal(2)-rot_punkt(2)];    %小车圆心到目标点的距离向量
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));  % 计算arcos 得到角度
    m=cross([w1;0],[w2;0]);
    if(m(3)<0)
        ang = -ang;     %定义目标在中轴线左侧角度为正
    end   
end
    