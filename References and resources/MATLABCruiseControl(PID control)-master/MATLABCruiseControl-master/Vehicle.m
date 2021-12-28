% *******************************************************************
% *   Vehicle.m
% *   Cruise Control
% *	Kareem Omar
% *	Student Researcher, CSPAR - UAH Class of 2017
% *
% *	8/30/2015
% *   This program is entirely my own work.
% *******************************************************************

% The Vehicle class propagates the physics of a traveling vehicle.
% It does not handle PID control in any way; that's contained
% in the PID module. This modular design means the PID controller
% is not dependent on the kind of vehicle, and vice versa.

classdef Vehicle < handle
    properties
        t = 0; % initial time [s]
        h = 0; % physics time step [s]
        v = 0; % initial v (modify w/ constructor) [m/s]
        steps = 0; % keep track of physics steps taken
        cur_pwr = 0; % current engine output [kW]
        min_pwr = 0; % min allowed power [kW]
        max_pwr = 149000; % max allowed power @ wheels [kW]
        m = 1542; % [kg]
        g = 9.81; % [m/s^2]
        A = 2.5; % [m^2]
        c_d = 0.34;
        % c_d = 0.24; % it's a Tesla Model S...
        rho = 1.2041; % [kg/m^3]
    end %properties
    
    methods
        function obj = Vehicle(v, h)
            if nargin == 2
                if isnumeric([v, h])
                    obj.v = v;
                    obj.h = h;
                else
                    error('Values must be numeric')
                end %if
            end %if
        end %function
        
        % RK4 propagator
        function r = stepPhysics(obj)
            k_1 = dv_dt(obj, obj.t, obj.v);
            k_2 = dv_dt(obj, obj.t+obj.h/2, obj.v + obj.h/2*k_1);
            k_3 = dv_dt(obj, obj.t+obj.h/2, obj.v + obj.h/2*k_2);
            k_4 = dv_dt(obj, obj.t+obj.h, obj.v + obj.h*k_3);
            obj.t = obj.h*obj.steps;
            obj.steps = obj.steps + 1;
            obj.v = obj.v + obj.h/6*(k_1+2*k_2+2*k_3+k_4);
            r = obj.v;
        end %function
        
        
        % energy at time t is kinetic plus potential
        % energy loss is frictional
        % energy input is engine
        % E = 1/2 * m * v^2 + m * g * h
        % dE/dt = m*v*dv/dt+m*g*dh/dt - 1/2*p*v^3*a*C_d + engine = 0
        % -mg(dh/dt)-.5pv^3a*c_d + engine = mv(dv/dt)
        % dv/dt = [engine - mg(dh/dt)-.5pv^3a*c_d] / mv
        function dvdt = dv_dt(obj, t, v)
            dvdt = (obj.cur_pwr - obj.m*obj.g*dh_dt(obj, t, v) - 0.5*obj.rho*v^3*obj.A*obj.c_d) / (obj.m*v);
        end %function
        
        function dvdt_nopower = dv_dt_nopower(obj, t, v)
            % prevent ode45() from letting v go negative
            if v < 0.1
                dvdt_nopower = 0;
            else
                dvdt_nopower = (-obj.m*obj.g*dh_dt(obj, t, v) - 0.5*obj.rho*v^3*obj.A*obj.c_d) / (obj.m*v);
            end %if
        end %function
        
        % add hills here!
        % dh/dt of 4 m/s corresponds to 11.5 degrees at 20 m/s,
        % which is quite a lot for cruise control. 2.5-3 is
        % a reasonable max
        function dhdt = dh_dt(~, ~, ~)
%         function dhdt = dh_dt(obj, t, v)
%             dhdt = 2.5;
%             dhdt = -0.4;
%             dhdt = 0.3*sin(t/20) - 0.1;
            dhdt = 0;
        end %function
                
    end %methods
end %classdef
