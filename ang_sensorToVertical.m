function ang = ang_sensorToVertical( sensor_vector,vertical_vector)  
    % ���������������������нǣ�����w1���Ϊ������w1�Ҳ�Ϊ��

    
    w1 = sensor_vector';  %�õ�С���˶�����
    w2 = vertical_vector';    %С��Բ�ĵ�Ŀ���ľ�������
    ang = acos(w1'*w2/(norm(w1)*norm(w2)));  % ����arcos �õ��Ƕ�
    m=cross([w1;0],[w2;0]);
    if(m(3)<0)
        ang = -ang;     %����Ŀ�������������Ƕ�Ϊ��
    end   
end
    