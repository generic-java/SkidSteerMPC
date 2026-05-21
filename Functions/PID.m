classdef PID < handle
    properties (Access = public)
        kP {mustBeNumeric}
        kI {mustBeNumeric}
        kD {mustBeNumeric}
    end
    properties (Access = private)
        I_curr = 0;
        last_error = 0;
        timer_started = 0;
    end
    methods
        % Constructor
        function self = PID(kP, kI, kD)
            self.kP = kP;
            self.kI = kI;
            self.kD = kD;
        end

        function output = calculate(self, measurement, setpoint)

            error = setpoint - measurement;

            if self.timer_started == 1
                dt = toc;
                tic;

                dErr_dt = (error - self.last_error) / dt;
                self.last_error = error;
    
                self.I_curr = self.I_curr + error * dt;

                output = self.kP * error + self.kI * self.I_curr + self.kD * dErr_dt;
            else
                tic;
                self.timer_started = 1;
                output = self.kP * error;
            end
        end
    end
end