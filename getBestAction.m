function a = getBestAction(Q, state, actions)
    
    %% ����˵�� ��״̬s ��ѡ����Ѷ���  a
    %{
       ����  Q��       Qֵ��
             state:    ״̬
             actions   �����б�
       ���  Q_value   ���Qֵ
             a         ���Qֵ��Ӧ�Ķ���
    %}
    n_actions = length(actions);
    status = find(Q(state,:), 1);   % ��ȡ��Ӧ״̬Q���һ��/��
      
    if isempty(status)
        
        a = randi(n_actions);        
    else
        
        [Q_value, a] = max(Q(state,:));       
    end
    
end