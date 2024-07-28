function a = getBestAction(Q, state, actions)
    
    %% 函数说明 在状态s 下选择最佳动作  a
    %{
       输入  Q：       Q值表
             state:    状态
             actions   动作列表
       输出  Q_value   最佳Q值
             a         最佳Q值对应的动作
    %}
    n_actions = length(actions);
    status = find(Q(state,:), 1);   % 提取对应状态Q表的一列/行
      
    if isempty(status)
        
        a = randi(n_actions);        
    else
        
        [Q_value, a] = max(Q(state,:));       
    end
    
end