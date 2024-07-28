function g = tanhDerivative(z)

    % The derivative of the hyperbolic tangent    双曲正切的导数
    
    g = ((4.*(cosh(z)).^2)./((cosh(2*z)+1).^2));
end