function y = PID_controller_fcn(reference, step_time, state, P_gain,I_gain,D_gain)
    kp = P_gain;
    ki = I_gain;
    kd = D_gain;
    persistent error error_pre error_i
    if isempty(error_pre)
        error_pre = 0;
        error_i = 0; 
    end
    error_p = reference - state;
    error_i = error_i + error_p*step_time;
    error_d = (error_p - error_pre)/step_time;
%     if abs(error_p) < 0.1 %errop_p 즉 레퍼런스와 현재 값의 차이가 너무 작으면 p_gain을 0으로 해서 컨트롤을 하지 않는다.
%                           %목표에 도달하지 않게 컨트롤 한다.
%         kp = 0;
%     end
%     if abs(error_i) < 0.05 %오차가 쌓이고있는 것을 바로 해소하는 것이 아닌 0.05 이상 쌓인 후 정상상태 오류 제거
%                            %steady state error 제거 
%         ki = 0;
%     end
    error = kp*error_p + ki*error_i + kd*error_d;
    error_pre = error_p;
    y = error;
end