function worker_punkt = moveworker(patch,tot_rot,velocity)   
    %% ����˵���� ƽ��������̬�ϰ�����ҷ������ǵ���������

    P = get(patch,'Vertices');
    r = ones(size(P));

    r = P(:,1:2) + velocity.*[cos(tot_rot)*r(:,1), sin(tot_rot)*r(:,2)];
    set(patch, 'XData', r(:,1), 'YData', r(:,2));

    tot_rot = (min(P(:,1)) + max(P(:,1)))/2;
    b = (min(P(:,2)) + max(P(:,2)))/2;
    worker_punkt = [tot_rot, b];   
end