% *******************************************************************
% *   PID.m
% *   Cruise Control
% *	Kareem Omar
% *	Student Researcher, CSPAR - UAH Class of 2017
% *
% *	8/30/2015
% *   This program is entirely my own work.
% *******************************************************************

% The PID class contains a modified (see below) PID controller.
% It does not contain vehicle physics at all.
% This modular design means the PID controller
% is not dependent on the kind of vehicle, and vice versa.

classdef PID < handle
    properties
        % We expect fewer calls to the controller
        % than to physics engines. Separately
        % keep a previous velocity and power for finite diff.
        prev_v = 0;
        prev_pwr = 0;            
        
        int_e = 0; % error integrator
        K_P = 0; % proportional gain
        K_I = 0; % integral gain
        K_D = 0; % derivative gain
        
        % Floor and ceiling of integral term to prevent
        % massive under/overshoots due to slow engine response or
        % slow drag-only deceleration
        min_I = 0;
        max_I = 45;
            
        % Max rates of integral term accum. in both neg and pos
        min_delta_I = -0.1;
        max_delta_I = 0.7;
        
        % Max power changes [W/s]
        min_delta_pwr = -80000;
        max_delta_pwr = 40000;
        
    end %properties
    methods
        function obj = PID(K_P, K_I, K_D)
            if nargin == 3
                if isnumeric([K_P K_I K_D])
                    obj.K_P = K_P;
                    obj.K_I = K_I;
                    obj.K_D = K_D;
                else
                    error('Values must be numeric')
                end %if
            end %if
        end %function
        
        function pwr = stepControl(obj, veh, set_point, dt)
            e = set_point - veh.v;

            dv_dt = (veh.v - obj.prev_v)/dt;
            obj.prev_v = veh.v;
            de_dt = -dv_dt;
            % This is normally ONLY TRUE if the set point
            % remains constant. In this application it does not;
            % however, we do not want the derivative corrector
            % producing an infinite power request if the set point
            % changes instantaneously. Thus we deliberately ignore
            % set point changes and only respond to dv/dt.
            % de_dt = d(set_point - v)/dt = -dv/dt.

            % Prevent excessive over/undershoots by limiting
            % the RATE at which the integral term accumulates.
            % This is a *modification* of canonical PID.
            % It greatly improves performance by allowing
            % a much higher integral constant than would
            % otherwise be possible without causing enormous
            % over/undershoots, thus improving controller
            % response.
            if e > obj.max_delta_I
                obj.int_e = obj.int_e + dt*obj.max_delta_I;  
            elseif e < obj.min_delta_I
                obj.int_e = obj.int_e + dt*obj.min_delta_I;    
            else
                obj.int_e = obj.int_e + dt*e;
            end %if
            
            % Also prevent excessive over/undershoots by limiting
            % the integral term itself.
            % Specifically: the controller does not slam the brakes to
            % to decelerate (i.e. min power is 0, not negative),
            % so this is not a truly linear system. The car
            % may not have extreme acceleration and decelerates very
            % slowly, which would otherwise result in a huge build-up
            % of historical error.
            if obj.int_e < obj.min_I, obj.int_e = obj.min_I; end
            if obj.int_e > obj.max_I, obj.int_e = obj.max_I; end

            pwr = obj.K_P*e + obj.K_I*obj.int_e + obj.K_D*de_dt;
            if pwr < veh.min_pwr, pwr = veh.min_pwr; end
            if pwr > veh.max_pwr, pwr = veh.max_pwr; end
            
            if pwr - obj.prev_pwr > (obj.max_delta_pwr * dt)
                pwr = obj.prev_pwr + (obj.max_delta_pwr * dt);
            end
            if pwr - obj.prev_pwr < (obj.min_delta_pwr * dt)
                pwr = obj.prev_pwr + (obj.min_delta_pwr * dt);
            end

            obj.prev_pwr = pwr;
            veh.cur_pwr = pwr;
        end %function
        
    end %methods
end %classdef