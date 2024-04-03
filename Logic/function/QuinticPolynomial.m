classdef QuinticPolynomial
    properties
        a0
        a1
        a2
        a3
        a4
        a5
    end
    
    methods
        function obj = QuinticPolynomial(xi, vi, ai, xf, vf, af, T)
            % QuinticPolynomial constructor
            obj.a0 = xi;
            obj.a1 = vi;
            obj.a2 = 0.5 * ai;
            
            A = [T^3, T^4, T^5;
                 3*T^2, 4*T^3, 5*T^4;
                 6*T, 12*T^2, 20*T^3];
            b = [xf - obj.a0 - obj.a1 * T - obj.a2 * T^2;
                 vf - obj.a1 - 2 * obj.a2 * T;
                 af - 2 * obj.a2];
            x = A\b;
            
            obj.a3 = x(1);
            obj.a4 = x(2);
            obj.a5 = x(3);
        end
        
        function x = calc_pos(obj, t)
            % Calculate position
            x = obj.a0 + obj.a1 * t + obj.a2 * t^2 + obj.a3 * t^3 + obj.a4 * t^4 + obj.a5 * t^5;
        end
        
        function v = calc_vel(obj, t)
            % Calculate velocity
            v = obj.a1 + 2 * obj.a2 * t + 3 * obj.a3 * t^2 + 4 * obj.a4 * t^3 + 5 * obj.a5 * t^4;
        end
        
        function a = calc_acc(obj, t)
            % Calculate acceleration
            a = 2 * obj.a2 + 6 * obj.a3 * t + 12 * obj.a4 * t^2 + 20 * obj.a5 * t^3;
        end
        
        function j = calc_jerk(obj, t)
            % Calculate jerk
            j = 6 * obj.a3 + 24 * obj.a4 * t + 60 * obj.a5 * t^2;
        end
    end
end