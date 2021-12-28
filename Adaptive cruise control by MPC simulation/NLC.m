function [C , Ceq] = NLC(dU , u , U_min , U_max)

C = [u + dU - U_max
    -u - dU + U_min];   
Ceq = [] ;
end