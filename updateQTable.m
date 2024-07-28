function Q = updateQTable(state, a, reward, Q, next_state, alpha, gamma, arrive_check)

	% Update values in the Q-table
  
    Q(state, a) = Q(state,a) + alpha*(reward +(1 - arrive_check)* gamma* max(Q(next_state,:)) - Q(state,a));
    % arrive_check 是到达目标点判断量
    % 

end