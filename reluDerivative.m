function g = reluDerivative(z)
    % Derivative of the ReLU activation function
    % ReLU������ĵ���

    g = double(z > 0);
end