function Q_est = computeQEstimate(nn_params, input_layer_size, hidden_layer_size, ...
                                output_layer_size,inputs,reward,gamma,sensor_lengde,arrive_check)
    
    %% 程序说明 计算下一时刻的最优Q值
    %{
    输入：nn_params : 神经网络权值矩阵 生成的数组向量
          
    输出
    
    %}
    
               
     Q_neste = nnFeedForward(nn_params, input_layer_size, hidden_layer_size, ...
                          output_layer_size, inputs,sensor_lengde);
    Q_est = reward + (1-arrive_check)*gamma*max(Q_neste);
    
end