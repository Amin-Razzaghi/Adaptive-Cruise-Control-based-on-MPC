% *******************************************************************
% *   cruise.m
% *   Cruise Control
% *	Kareem Omar
% *	Student Researcher, CSPAR - UAH Class of 2017
% *
% *	8/30/2015
% *   This program is entirely my own work.
% *******************************************************************
% 
% Cruise Control is a PID controller (or PI - derivative
% term not required for this application), along with the
% required physics simulation (i.e. hills, drag) for demo purposes
% 
% This controller is optimized for normal car speeds, i.e. < 50 m/s.
% Of course, the controller may perform suboptimally far above this.
%
% Experiment with the parameters below, or perform advanced
% experimentation or modification of the PID system in PID.m.
%
% Add 'hills' with the dh_dt function in Vehicle.m.

V_init = 25; % initial velocity [m/s]
font_size = 18; % [pt]
h_physics = 0.05; % physics time step [s]
physics_ticks_per_control_tick = 4; % call PID every x physics ticks
T_max = 200; % end time [s]

% PID gains:
K_P = 22000;    % proportional gain
K_I = 2200;     % integral gain
K_D = 0;        % derivative gain

numTicks = floor(T_max/h_physics);
% ------adjust this array to configure setpoints as desired------
% set_points = [V_init*ones(1, floor(numTicks/4)), 15*ones(1, floor(numTicks/7)), 29*ones(1, floor(numTicks/4))];
% set_points = [];
set_points = [V_init*ones(1, floor(numTicks/9)), 37*ones(1, floor(numTicks/6)), 34*ones(1, floor(numTicks/7)), 25*ones(1, floor(numTicks/3)), 10*ones(1, floor(numTicks/7))];
if numel(set_points) < numTicks + 1
    set_points = [set_points, 29*ones(1, numTicks + 1 - numel(set_points))];
end %if
%----------------------------------------------------------------

% --------- END OF USER-CONFIGURABLE PARAMETERS ---------


veh = Vehicle(V_init, h_physics);
pid = PID(K_P, K_I, K_D);
clc
disp(veh), disp(pid)

% reference integrator with no engine power, i.e. coasting
% also enable plotting at bottom of file
[T_nopower, V_nopower] = ode45(@veh.dv_dt_nopower, [0 T_max], V_init);

% preallocate
T = zeros(1, numTicks);
V = zeros(1, numTicks);
pwr = zeros(1, numTicks);
h_control = h_physics*physics_ticks_per_control_tick;

for i=1:numTicks+1
    V(i) = veh.stepPhysics();
    T(i) = veh.t;
    if ~mod(i, physics_ticks_per_control_tick)
        pid.stepControl(veh, set_points(i), h_control);
    end %if
    pwr(i) = veh.cur_pwr;
end %for

KW_PER_W = 1e-3;
figure('NumberTitle', 'off', 'Name', 'PID Cruise Control - Requested Engine Power', 'units','normalized','outerposition',[0 0 1 1])
clf, hold on
set(gca,'FontSize',font_size)
title({'PID Cruise Control', 'Requested Engine Power'})
xlabel('Time [s]'), ylabel('Req. Power [kW]')
max_kw = veh.max_pwr*KW_PER_W;
plot([min(T), max(T)], [max_kw, max_kw], 'r', 'LineWidth', 1.5)
plot(T, pwr*KW_PER_W, 'b', 'LineWidth', 1.5)
ylim([0 veh.max_pwr*KW_PER_W+5])
legend('Maximum Engine Power', 'Location', 'Best')

figure('NumberTitle', 'off', 'Name', 'PID Cruise Control - Velocities', 'units','normalized','outerposition',[0 0 1 1])
clf, hold on
set(gca,'FontSize',font_size)
title({'PID Cruise Control', 'Velocities'})
xlabel('Time [s]'), ylabel('Speed [m/s]')
plot(T, set_points, 'r')
plot(T, V, 'b', 'LineWidth', 2)
plot(T_nopower, V_nopower, 'g', 'LineWidth', 2)
ylim([0 max(V)+1])
legend('Set Point', 'Cruise Control', 'Coasting', 'Location', 'Best')