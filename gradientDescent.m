function w = gradientDescent( nn_params, w_grad, alpha, ...
                                          input_layer_size, ...
                                          hidden_layer_size, ...
                                          output_layer_size)       
    %% ����˵�� DQN������ Ȩ�ظ���  
    %{
       ����  nn_params             ���������
             w_grad                ����Ȩ��
             alpha                 ����
             num_iters             
             input_layer_size      �������Ԫ����
             hidden_layer_size     ���ز���Ԫ����
             output_layer_size     �������Ԫ����
    
       ���  Q_value   ���Qֵ
             a         ���Qֵ��Ӧ�Ķ���
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