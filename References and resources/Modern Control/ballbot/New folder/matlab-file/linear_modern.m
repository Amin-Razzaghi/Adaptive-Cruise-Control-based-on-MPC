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

Dx = IBx*IM + IM*Ib - 2*IM*Ib*n + IBx*Ib*n*n + IM*Ib*n*n + IM*L*L*MB + IM*MB*Rb*Rb + IM*Mb*Rb*Rb + Ib*L*L*MB*n*n + IBx*MB*n*n*Rb*Rb + IM*MB*n*n*Rb*Rb + IBx*Mb*n*n*Rb*Rb + IM*Mb*n*n*Rb*Rb + 2*IM*L*MB*Rb - 2*IM*MB*n*Rb*Rb - 2*IM*Mb*n*Rb*Rb + L*L*MB*Mb*n*n*Rb*Rb - 2*IM*L*MB*n*Rb ; 
Dy = IBy*IM + IM*Ib - 2*IM*Ib*n + IBy*Ib*n*n + IM*Ib*n*n + IM*L*L*MB + IM*MB*Rb*Rb + IM*Mb*Rb*Rb + Ib*L*L*MB*n*n + IBy*MB*n*n*Rb*Rb + IM*MB*n*n*Rb*Rb + IBy*Mb*n*n*Rb*Rb + IM*Mb*n*n*Rb*Rb + 2*IM*L*MB*Rb - 2*IM*MB*n*Rb*Rb - 2*IM*Mb*n*Rb*Rb + L*L*MB*Mb*n*n*Rb*Rb - 2*IM*L*MB*n*Rb ; 


Ax_31 = g*L*MB*(IM + Ib*n*n + MB*n*n*Rb*Rb + Mb*n*n*Rb*Rb)/Dx ; 
Ax_41 = -g*L*MB*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb)/Dx ;
Ax_33 = -Meu_Bg*(IM + Ib*n*n  + MB*n*n*Rb*Rb + Mb*n*n*Rb*Rb)/Dx ;
Ax_43 = Meu_Bg*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*n*Rb*Rb + L*MB*n*Rb)/Dx ;
Ax_34 = (Meu_Bb + ((Kb*Kt)/Rm)*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb))/Dx ;
Ax_44 = -(Meu_Bb + ((Kb*Kt)/Rm)*(IBx + IM + Ib + L*L*MB +  MB*Rb*Rb + Mb*Rb*Rb + 2*L*MB*Rb))/Dx ;

Ay_44 = -(Meu_Bb + ((Kb*Kt)/Rm)*(IBy + IM + Ib + L*L*MB +  MB*Rb*Rb + Mb*Rb*Rb + 2*L*MB*Rb))/Dy ;
Ay_33 = -Meu_Bg*(IM + Ib*n*n  + MB*n*n*Rb*Rb + Mb*n*n*Rb*Rb)/Dy ;
Ay_34 = (Meu_Bb + ((Kb*Kt)/Rm)*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb))/Dy ;
Ay_43 = Meu_Bg*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*n*Rb*Rb + L*MB*n*Rb)/Dy ;
Ay_31 = g*L*MB*(IM + Ib*n*n + MB*n*n*Rb*Rb + Mb*n*n*Rb*Rb)/Dy ; 
Ay_41 = -g*L*MB*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb)/Dy ;


Bx_31 = -Kt*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb)/(Dx*Rm) ;
Bx_41 = Kt*(IBx + IM + Ib + L*L*MB +  MB*Rb*Rb + Mb*Rb*Rb + 2*L*MB*Rb)/(Dx*Rm) ; 

By_31 = -Kt*(IM + Ib*n + MB*n*Rb*Rb + Mb*n*Rb*Rb + L*MB*n*Rb)/(Dy*Rm) ;
By_41 = Kt*(IBy + IM + Ib + L*L*MB +  MB*Rb*Rb + Mb*Rb*Rb + 2*L*MB*Rb)/(Dy*Rm) ; 



A = [ 0 0 1 0 ; 0 0 0 1; Ax_31 0 Ax_33 Ax_34 ; Ax_41 0 Ax_43 Ax_44]
B = [ 0  ; 0  ; Bx_31  ; Bx_41  ]
C = [ 0 1 0 0]

C2 = [ 1 0 0 0];
D = 0;
%A = [ 0 0 0 0 1 0 0 0 ; 0 0 0 0 0 1 0 0 ; 0 0 0 0 0 0 1 0 ; 0 0 0 0 0 0 0 1; Ax_31 0 0 0 Ax_33 Ax_34 0 0 ; 0 0 Ay_31 0 0 0 Ay_33 Ay_34 ; Ax_41 0 0 0 Ax_43 Ax_44 0 0 ; 0 0 Ay_41 0 0 0 Ay_43 Ay_44]
%B = [ 0 0 ; 0 0; 0 0; 0 0; Bx_31 0 ; Bx_41 0 ; 0 By_31; 0 By_41]


% state feedback
k = place(A,B,[-2,-3,-10,-15]) ; 
A_new = A - B*k ; 
