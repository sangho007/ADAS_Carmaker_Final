classdef QuarticPolynomial
    properties
        a0
        a1
        a2
        a3
        a4
    end
    
    methods
        function obj = QuarticPolynomial(xi, vi, ai, vf, af, T)
            % Constructor
            obj.a0 = xi;
            obj.a1 = vi;
            obj.a2 = 0.5 * ai;
            
            A = [3*T^2, 4*T^3; 6*T, 12*T^2];
            b = [vf - obj.a1 - 2*obj.a2*T; af - 2*obj.a2];
            x = A\b; % MATLAB's way to solve linear equations
            
            obj.a3 = x(1);
            obj.a4 = x(2);
        end
        
        function x = calc_pos2(obj, t)
            x = obj.a0 + obj.a1 * t + obj.a2 * t^2 + obj.a3 * t^3 + obj.a4 * t^4;
        end
        
        function v = calc_vel2(obj, t)
            v = obj.a1 + 2 * obj.a2 * t + 3 * obj.a3 * t^2 + 4 * obj.a4 * t^3;
        end
        
        function a = calc_acc2(obj, t)
            a = 2 * obj.a2 + 6 * obj.a3 * t + 12 * obj.a4 * t^2;
        end
        
        function j = calc_jerk2(obj, t)
            j = 6 * obj.a3 + 24 * obj.a4 * t;
        end
    end
end