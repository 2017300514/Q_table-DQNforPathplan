function w = gradientDescent( nn_params, w_grad, alpha, ...
                                          input_layer_size, ...
                                          hidden_layer_size, ...
                                          output_layer_size)       
    %% 程序说明 DQN神经网络 权重更新  
    %{
       输入  nn_params             神经网络参数
             w_grad                网络权重
             alpha                 参数
             num_iters             
             input_layer_size      输入层神经元个数
             hidden_layer_size     隐藏层神经元个数
             output_layer_size     输出层神经元个数
    
       输出  Q_value   最佳Q值
             a         最佳Q值对应的动作
    %}
    % Weight matrices of the Neural Network                                               
    w1 = reshape(nn_params(1:hidden_layer_size*(input_layer_size + 1)), ...
                 hidden_layer_size, (input_layer_size + 1));
    
    w2 = reshape(nn_params((1 + (hidden_layer_size*(input_layer_size + 1))):end), ...
                 output_layer_size, (hidden_layer_size + 1));

    % Partial derivatives
    w1_grad = reshape(w_grad(1:hidden_layer_size*(input_layer_size + 1)), ...
                 hidden_layer_size, (input_layer_size + 1));

    w2_grad = reshape(w_grad((1 + (hidden_layer_size*(input_layer_size + 1))):end), ...
                 output_layer_size, (hidden_layer_size + 1));
         

    w1 = w1 - alpha.*w1_grad;   
    w2 = w2 - alpha.*w2_grad;       
           
    w = [w1(:); w2(:)];

end