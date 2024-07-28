function g = reluDerivative(z)
    % Derivative of the ReLU activation function
    % ReLU激活函数的导数

    g = double(z > 0);
end