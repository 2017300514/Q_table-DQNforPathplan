function [C, grad] = nnBackPropagation(nn_params, input_layer_size, ...
                                    hidden_layer_size, output_layer_size, ...
                                    actions, inputs, a, Q_est,sensor_length)

    w1 = reshape(nn_params(1:hidden_layer_size * (input_layer_size + 1)), ...
        hidden_layer_size, (input_layer_size + 1));

    w2 = reshape(nn_params((1 + (hidden_layer_size * (input_layer_size + 1))):end), ...
        output_layer_size, (hidden_layer_size + 1));

    C = 0;
    w1_grad = zeros(size(w1));
    w2_grad = zeros(size(w2));
    n_actions = length(actions);
    I = eye(n_actions);

    % Input layer
    x = inputs./(sensor_length*10);    % Scale the input data between 0 and 1   

    % Feed forward
    L1 = [1; x'];
    z2 = w1*L1;
    % L2 = [1; tanhActivation(z2)];
    L2 = [1; reluActivation(z2)];
    
    z3 = w2*L2;
    % Q = tanhActivation(z3);
    Q = reluActivation(z3);
    
    C = (1/2)*( Q_est - Q(a) ).^2;
    
    delta_3 = (Q(a) - Q_est).*I(:,a);
   
    % temp = (w2'*delta_3).*tanhDerivative([1;z2]);
    temp = (w2'*delta_3).*reluDerivative([1;z2]);
    
    delta_2 = temp(2:end);                                       

    w1_grad = delta_2*L1';  
    w2_grad = delta_3*L2'; 

    grad = [w1_grad(:) ; w2_grad(:)];
    
end
