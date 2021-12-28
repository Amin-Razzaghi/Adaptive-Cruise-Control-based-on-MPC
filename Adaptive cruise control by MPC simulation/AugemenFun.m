function [A , B , C] = AugemenFun(Am , Bm , Cm)
Ne = size(Am ,1)  ;  % number of eigenvalues
No = size(Cm , 1) ;  % number of outputs
% Ni = size(Bm , 2) ;  % number of inputs
Om = zeros(No , Ne) ;
Im = eye(No);
A = [Am ,  Om'
    Cm*Am  Im] ;
B = [Bm
    Cm*Bm] ;
C = [Om , Im ] ;
end