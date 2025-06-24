classdef PIDController
    properties
        Kp
        Ki
        Kd
    end
    
    methods
        function obj = PIDController(Kp, Ki, Kd)
            obj.Kp = Kp(:);
            obj.Ki = Ki(:);
            obj.Kd = Kd(:);
        end
        
        function tau = computeControl(obj, e_pos, e_int, e_vel)
            % Ensure that the errors are column vectors
            e_pos = e_pos(:);
            e_vel = e_vel(:);
            e_int = e_int(:);
            
            for i = 1:length(e_pos)
                tau(i,1) = obj.Kp(i) * e_pos(i) + obj.Ki(i) * e_int(i) + obj.Kd(i) * e_vel(i);
                % tau(i,1) = obj.Kp(i) * e_pos(i) + obj.Kd(i) * e_vel(i);
            end
        end
    end
end
