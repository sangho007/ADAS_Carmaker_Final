classdef PolynomialFitting_class
    properties
        nd; % Degree of the polynomial
        np; % Number of points
        A;  % Design matrix
        b;  % Vector of y-values
        coeff; % Coefficients of the polynomial
        lambda; % Regularization parameter
    end
    
    methods
        function obj = PolynomialFitting_class(num_degree, num_points, lambda)
            obj.nd = num_degree;
            obj.np = num_points;
            obj.A = zeros(obj.np, obj.nd+1);
            obj.b = zeros(obj.np, 1);
            obj.coeff = zeros(num_degree+1, 1);
            obj.lambda = lambda; % Set the regularization parameter
        end
        
        function obj = fit(obj, points)
            for i = 1:obj.np
                for j = 1:obj.nd+1
                    obj.A(i, j) = points(i, 1)^(obj.nd-j+1);
                end
                obj.b(i, 1) = points(i, 2);
            end
            
            % Apply regularization
            regularization_matrix = obj.lambda * eye(obj.nd + 1);
            regularization_matrix(1, 1) = 0; % Typically, we don't regularize the bias term
            
            % Compute the coefficients using the regularized least squares formula
            obj.coeff = (obj.A' * obj.A + regularization_matrix) \ (obj.A' * obj.b);
        end
        function y = evaluate(obj, x)
            y = polyval(flip(obj.coeff), x);
        end
        
        function dy = differentiate(obj, x)
            coeff_diff = polyder(flip(obj.coeff));
            dy = polyval(coeff_diff, x);
        end

    end
end
