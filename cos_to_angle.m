function ang = cos_to_angle( car,rot_punkt,goal,sensor_length)  
    %% ����˵�� ��������������С����Ŀ�����߽�
    %{
        ���룺 car  С���봫�������� patch����
        rot_punkt С������Բ��
        goal Ŀ���λ��
        ����� ang ���߽�
    %}
    %%
    A = get(car(4),'Vertices');   % ��ȡС�������ߴ�������ģ�Ͷ˵㣬
    w1 = [A(2,1)-A(1,1);A(2,2)-A(1,2)];  %�õ�С���˶�����
    w2 = [goal(1)-rot_punkt(1);goal(2)-rot_punkt(2)];    %С��Բ�ĵ�Ŀ���ľ�������
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));  % ����arcos �õ��Ƕ�
    m=cross([w1;0],[w2;0]);
    if(m(3)<0)
        ang = 2*pi-ang;     %����Ŀ�������������Ƕ�Ϊ��
%         ang = -ang;
    end
%     ang = ang*(sensor_length*10)/3.1416;
     ang = ang*(sensor_length*10)/(2*3.1416);
end
    