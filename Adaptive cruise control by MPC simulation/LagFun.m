function [ Al , L0 ] = LagFun( N , a  )

L0(1,1) = 1 ;
v(1,1) = a;
for i=2:N
    v(i,1) = (-a)^(i-2) * (1 - a^2) ;
    L0(i,1)= (-a)^(i-1) ;
end

L0 = (1-a^2)^0.5 * L0;
Al(: , 1) = v;

for i=2:N
    Al( : , i) = [zeros(i-1 , 1); v(1:N-i+1)];
end     
end


