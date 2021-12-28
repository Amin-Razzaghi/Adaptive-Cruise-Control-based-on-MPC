clear ;
close all;
clc

% Define the sample time, |Ts|, and simulation duration, |t|, in seconds.
t0 = 0;
Ts = 0.1;
Tf = 100;
t = t0:Ts:Tf;               
Nt = numel(t);
% Specify the initial position and velocity for the two vehicles.

%x0_lead = 0;               %Initial position of lead car (m)
%v0_lead = 0;               %Initial velocity of lead car (m/s)

%x0_ego = 0;                %Initial position of ego car (m)
%v0_ego = 0;                %Initial velocity of ego car (m/s)

% The safe distance between the lead car and the ego car is a function
% of the ego car velocity, $V_{ego}$:
%
% $$ D_{safe} = D_{default} + T_{gap}\times V_{ego} $$
%
% where $D_{default}$ is the standstill default spacing and $T_{gap}$ is
% the time gap between the vehicles. Specify values for $D_{default}$, in
% meters, and $T_{gap}$, in seconds.
t_gap = 1.4;
D_default = 10;

% Specify the driver-set velocity in m/s.
v_set = 30;

% Considering the physical limitations of the vehicle dynamics, the
% acceleration is constrained to the range  |[-3,2]| (m/s^2).
a_max = 2; da_max = 0.15;
a_min = -3; da_min = -0.2;



% the relationship between the actual acceleration and the desired 
% acceleration of the host vehicle satisfies the following conditions 
% 
%  $$ a(k+1) = (1-\frac{Ts}{\tau}) \times a(k) + \frac{Ts}{\tau} \times u(k)$$
%  
% where $ \tau $ is the time lag of the ACC system
tau = 0.3;


%Np = 20 ;          % Prediction Horizon
%Nc = 20 ;          % Control Horizon

%% Examples
%  In this section we want to try to specify the various parameters 
%   of the machine for different simulation


%EX.1
% N = 5;
% Np = 20 ;          % Prediction Horizon
% Nc = 5 ;          % Control Horizon
% x0_ego  = 0;
% v0_ego  = 0;
% x0_lead = 50;
% v0_lead = 15;
% a_lead = 0.3*sin(2*pi*0.03*t);  % Acceleration of lead car is a disturbance for our plant;
% [lead_car_position , lead_car_velocity] = lead_car_simulation(x0_lead,v0_lead,a_lead,t,Ts ,tau);

% EX.2
N = 5;
Np = 20 ;          % Prediction Horizon
Nc = 15 ;          % Control Horizon
x0_ego  = 0;
v0_ego  = 0;
x0_lead = 20;
v0_lead = 5;
a_lead = [1*(1-exp(-0.5*t(1:floor(Nt/5)))) ,0.5+0.5*exp(-0.5*t(1:floor(Nt/5))) , -0.5+exp(-0.5*t(1:floor(Nt/5))) ,-0.5*exp(-0.5*t(1:floor(Nt/5))) , zeros(1,floor(Nt/5)+1)];  
[lead_car_position , lead_car_velocity] = lead_car_simulation(x0_lead,v0_lead,a_lead,t,Ts ,tau);

% % % EX.3
% Np = 20 ;          % Prediction Horizon
% Nc = 15 ;          % Control Horizon
% x0_ego  = 0;
% v0_ego  = 0;
% x0_lead = 1500;
% v0_lead = 0;
% a_lead = zeros(1,Nt) ;
% [lead_car_position , lead_car_velocity] = lead_car_simulation(x0_lead,v0_lead,a_lead,t,Ts ,tau);

%% Car State Space Model

Am=[1   Ts   0.5*Ts^2
    0   1      Ts
    0   0     1-Ts/tau ];

Bm=[0 ; 0 ; Ts/tau];

Cm=[1   0  0
    0   1  0];


n = size(Am , 1) ;  % number of eigenvalues
q = size(Cm , 1) ;  % number of outputs
m = size(Bm , 2) ;  % number of inputs


[A , B , C] = AugemenFun(Am , Bm , Cm) ;

a = 0.5 ;
[Al , L0] = LagFun(N,a);
L = zeros( N , Nc ); 
L( : , 1) = L0 ; 
for i = 2:Nc
    L(:,i) = Al*L(: , i-1) ; 
end


F = zeros(q*Np , size(A , 1)) ;
for i = 1:Np
    F(q*i-q+1:q*i , :) = C * A^i ;
end

PHI2 = zeros(q * Np , N);
for i = 1:Np
    summ = 0;
    for j = 1:i 
        if j == Nc +1  
            break
        end    
        summ = summ + C*A^(i-j) * B * L(:,j)' ;
        
    end
    PHI2((2*i)-1:(2*i) ,:) = summ;
end



PHI = zeros(q * Np , m*Nc);
for i = 1:Np
    for j = 1:i
        PHI(q*i-q+1:q*i , m*j-m+1:m*j) = C * A^(i-j) * B  ;
    end
end
PHI = PHI( : , 1:Nc*m) ;

%%  Mpc sim
du = zeros(m , Nt) ;
u = zeros(m , Nt) ;
x = zeros(size(A , 1) , Nt);    x(:,1:2)=[0 Ts*v0_ego;0 0;0 0;x0_ego x0_ego;v0_ego v0_ego];
y = zeros(q , Nt);              y(:,1:2) = [x0_ego x0_ego;v0_ego v0_ego];

dU = zeros(Nc,1);

% for lag functions 
etha = zeros(N,1);
% till here 

% disturbances 
DIST = 1; % 1:add 
          % 0:don't add
dist = -1.*(t>40).*(t<43) -0.5.*(t > 60) .*(t<65) ;            

dU_min = da_min*ones(Nc,1);
dU_max = da_max*ones(Nc,1);

U_min = a_min*ones(Nc,1);
U_max = a_max*ones(Nc,1);

for  i = 2:Nt-1
    FreeResponse = F * x(: , i) + DIST * dist(i) ;
    
    
    Y_des = feedbacksystem(y(1,i),y(2,i),lead_car_position(i),lead_car_velocity(i),v_set,D_default,t_gap);
    
    NonLinearCon = @(U)  NLC(U , u(i-1) , U_min , U_max) ;
   Cost = @(dU) norm(repmat(Y_des,Np,1) - PHI * dU -FreeResponse , 1) + norm(dU , 2) ;
   [dU] = fmincon(Cost ,dU,[],[],[],[],dU_min,dU_max,NonLinearCon) ;     
    du(: , i)= dU(1:m);
     u(i) = u(i-1) + du(i) ;
  
    %for lag functions
%     Cost = @(etha) norm(repmat(Y_des,Np,1) - PHI2 * etha -FreeResponse , 1) + norm(etha , 2) ;
%     [etha] = fmincon(Cost ,etha,[],[],[],[],[],[]) ;
% 
%      du(: , i)= L(: , 1)' *etha;
%      if du(: , i) > da_max
%          du(: , i) = da_max-0.05;
%      elseif du(: , i) < da_min   
%          u(: , i) = da_min ;
%      end    
%     u(i) = u(i-1) + du(i) ;
%      if u(i) > a_max
%          u( i) = a_max;
%      elseif u(i) < a_min   
%          u(i) = a_min;
%      end    
     
     % till here 
    
    x(: , i+1) = A * x(: , i) + B * du(: ,i) ;
    y(: , i+1) = C * x(: , i+1) + DIST * dist(i)  ;
    y(: , i+1) = y(: , i+1)  ; % adding disturbances to V
end

%% plots
figure(2);
subplot(3,2,1) ;
plot(t,lead_car_position, t,y(1 , :), 'LineWidth' , 2) ; hold on %,  t,lead_car_position-y(1 , :)
xlabel('Time  (second)') ;
ylabel('POSITION (m)') ;
title('Position of lead & ego car in road') ;
grid on
legend('lead car position','ego car position') ; %,'\DeltaX'

subplot(3,2,2) ;
plot(t,lead_car_velocity,  t,y(2 , :),  t,lead_car_velocity-y(2,:), t,v_set*ones(1,Nt),'k--', 'LineWidth' , 2) ; hold on
xlabel('Time  (second)') ;
ylabel('VELOCITY (m/s)') ;
title('Velocity of lead & ego car') ;
grid on
legend('V_{lead}','V_{ego}','V_{rel}','V_{set}') ;


subplot(3,2,3) ;
plot(t , a_lead,t  , u , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('ACCELERATION (m/s^2)') ;
title('Acceleration of lead & ego car') ;
grid on
legend( 'a_{lead}' , 'a_{ego}') ;

subplot(3,2,4) ;
plot( t  , du(1 , :) , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('Acceleration changes (du) ') ;
title('Control effort') ;
grid on
legend('du') ;

subplot(3,2,[5 6]) ;
plot(t  , D_default + t_gap*y(2,:) , t,lead_car_position-y(1 , :) , 'LineWidth' , 2) ; hold on
xlabel('Time (second)') ;
ylabel('\DeltaX (m)') ;
title('safety Inter-distance') ;
grid on
legend('safe distance between to vehicle','real distance between to vehicle') ;



