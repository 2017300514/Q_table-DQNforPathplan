function ang = cos_to_angle( car,rot_punkt,goal,sensor_length)  
    %% 程序说明 ：函数用来计算小车与目标视线角
    %{
        输入： car  小车与传感器整体 patch对象
        rot_punkt 小车车体圆心
        goal 目标点位置
        输出： ang 视线角
    %}
    %%
    A = get(car(4),'Vertices');   % 获取小车中轴线处传感器模型端点，
    w1 = [A(2,1)-A(1,1);A(2,2)-A(1,2)];  %得到小车运动方向
    w2 = [goal(1)-rot_punkt(1);goal(2)-rot_punkt(2)];    %小车圆心到目标点的距离向量
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));  % 计算arcos 得到角度
    m=cross([w1;0],[w2;0]);
    if(m(3)<0)
        ang = 2*pi-ang;     %定义目标在中轴线左侧角度为正
%         ang = -ang;
    end
%     ang = ang*(sensor_length*10)/3.1416;
     ang = ang*(sensor_length*10)/(2*3.1416);
end
    