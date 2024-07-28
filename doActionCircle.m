function [tot_rot,rot_punkt] = doActionCircle(action, tot_rot,v_loit, h_car)

    % Function for moving the vehicle in the simulation
    %{ 
    ���룺 ������ţ�������Ŀǰ���ٶȷ��򣬴�С���������ģ��λ�ã�����С��ģ�ͺʹ�����ģ��
    ����� С�����ٶȷ��򣬺�����λ��(ģ������Բ��Բ��)
    ע�⣬theta ֵ���뵽 rotate ��������matlab��װ�õģ�
          ��ֵ��ʱ�룬��ֵ˳ʱ��,��λ�ǻ��ȣ�����moveCarCircle.m����ת��Ϊ�ȣ�
    %}
    % �������Ҫ���ĸĶ���
    %{
    1. �任�˶��ռ䣬�˸��˶��ռ�ά��
    2. �ı��˶��ٶȺ��˶��ٶȸı�ķ�ʽ����Ȼ��˲��ת�Ƶķ�ʽ����
    3. ��� ����ǣ�λ��
    %}


    v = v_loit;
    a = action;
    
    switch(a)
        
        % Move forward
        case 1
            theta = 0;
        
        % Turn left    
        case 2
            theta = pi/3;
        % Turn right
        case 3
            theta = -pi/3;
    end

    tot_rot = tot_rot + theta;          % Total rotation

    rot_punkt = moveCarCircle(v, tot_rot, theta, h_car(1));
    moveSensor(v, rot_punkt, tot_rot, theta, h_car(2:end)); 
end