function Q_est = computeQEstimate(nn_params, input_layer_size, hidden_layer_size, ...
                                output_layer_size,inputs,reward,gamma,sensor_lengde,arrive_check)
    
    %% ����˵�� ������һʱ�̵�����Qֵ
    %{
    ���룺nn_params : ������Ȩֵ���� ���ɵ���������
          
    ���
    
    %}
    
               
     Q_neste = nnFeedForward(nn_params, input_layer_size, hidden_layer_size, ...
                          output_layer_size, inputs,sensor_lengde);
    Q_est = reward + (1-arrive_check)*gamma*max(Q_neste);
    
end