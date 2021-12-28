MB = 0.77 ;
L = 0.128 ;
Mb = 0.01 ;
Rb = 0.026 ; 
IBx = 0.003677;
IBy = 0.003927 ;
Ib = 0;
n = (-30/52)*(-20/12);
IM = 2*10^(-5);
Meu_Bb = 0.0022 ;
Meu_Bg = 0;
g = 10 ;

% Servo_Motor 
Kt = 0.317 ;
Kb = 0.468 ; 
Rm = 6.69 ; 





syms tetha_x ;
syms x_dot ; 
syms phi ;
syms phi_dot ;

Mx_11 = IBx + IM + Ib + L*L*MB + MB*Rb*Rb + Mb*Rb*Rb + 2*L*MB*Rb*cos(tetha_x);
Mx_12 = IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb*cos(tetha_x);
Mx_21 = IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb*cos(tetha_x);
Mx_22 = IM + Ib*n*n + MB*n*n*Rb*Rb + Mb*n*n*Rb*Rb ; 

M =[Mx_11 , Mx_12 ; Mx_21 , Mx_22] ;

Rx = [ -L*MB*Rb*sin(tetha_x)*(x_dot)*(x_dot) - g*L*MB*sin(tetha_x) ; -L*MB*n*Rb*x_dot*x_dot*sin(tetha_x)] ;
    

syms v ;
Fx = [ -Meu_Bg*x_dot ; Kt*v/Rm - Meu_Bb*phi_dot - (Kb*Kt*phi_dot)/Rm ] ;

final = inv(M)*(Fx - Rx) 


