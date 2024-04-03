function u = PID_controller(reference, measure,step_time,P_gain, I_gain, D_gain)
    persistent flag error_d error_i error_prev

    if isempty(flag)
        error_d = 0;
        error_i = 0;
        error_prev = reference - measure;
    end

    error = (reference - measure);

    error_d = (error - error_prev) / step_time;

    error_i = error_i + error * step_time;

    if error_i > 10
        error_i = 0;
    end

    error_prev = error;

    u = P_gain * error + I_gain * error_i + D_gain * error_d;
end