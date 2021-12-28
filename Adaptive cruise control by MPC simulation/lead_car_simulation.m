function [position , velocity] = lead_car_simulation(x0,v0,a,t,Ts ,tau)
%% lead car dynamic system and state variable

A = [1  Ts   0.5*Ts^2
    0   1     Ts
    0   0    1-Ts/tau ];                % system matrix

B = [0 ; 0 ; Ts/tau ];                  % input matrix

C = [1   0  0
    0   1  0];                          % output matrix

n = size(A ,1);                         % number of eigenvalues
q = size(C ,1);                         % number of output

Nt = numel(t);                          % number of samples 
x = zeros(n , Nt);                      % state variable vector that we want specify it in each time sample 
x(:,1) = [x0;v0;0];                     % initial condition of lead car specified
y = zeros(q , Nt);                      % output vector that we want specify it in each time sample 
y(:,1) = [x0;v0];
u = a;                                  % for Example: a_lead = 0.2*sin(2*pi*0.1*t) or every disturbance you want ;

% we have acceleration of lead car as input and we try to specify position
% and velocity of lead car
for i = 1:Nt-1
    x(: , i+1) = A*x(:,i) + B*u(i);
    y(: , i+1) = C*x(:,i+1);
end
position = y(1,:);
velocity = y(2,:);

%% plot lead car position , velocity and acceleration in figure 1
figure(1)
subplot(3,1,1)
plot(t  , x(1 , :) , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('position (m)') ;
title('Preceding vehicle position') ;
grid on
legend('lead car position') ;

subplot(3,1,2)
plot(t  , x(2 , :) , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('velocity (m/s)') ;
title('Preceding vehicle velocity') ;
grid on
legend('lead car velocity') ;

subplot(3,1,3)
plot(t  , x(3 , :) , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('acceleration (m/s^2)') ;
title('Preceding vehicle acceleration') ;
grid on
legend('lead car acceleration') ;

end