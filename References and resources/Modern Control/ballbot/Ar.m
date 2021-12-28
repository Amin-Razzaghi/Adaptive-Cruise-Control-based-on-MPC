% F  = @(t, x) [ 0.5*(-17.17*x(1) + 103.79*x(1)^2 -  229.62*x(1)^3 + 226.31*x(1)^4 - 83.72*x(1)^5 + x(2)); 0.2*(-x(1) - 1.5*x(2) + 1.2)];

F1  = @(t, x) [ x(2); -x(1) + x(2)*(1 - x(1)^2 - 0.1*x(1)^4)];

F2  = @(t, x) [ -x(2); +x(1) - x(2)*(1 - x(1)^2 - 0.1*x(1)^4)];


x1 = linspace(-3,3,20);
x2 = linspace(-3,3,20);

% creates two matrices one for all the x-values on the grid, and one for
% all the y-values on the grid. Note that x and y are matrices of the same
% size and shape, in this case 20 rows and 20 columns
[x,y] = meshgrid(x1,x2);

u = zeros(size(x));
v = zeros(size(y));
% % we can use a single loop over each element to compute the derivatives at
% % each point (y1, y2)
t=0; % we want the derivatives at each point at t=0, i.e. the starting time
for i = 1:numel(x)
    xdot = F(t,[x(i); y(i)]);
    u(i) = xdot(1);
    v(i) = xdot(2);
end

quiver(x,y,u,v,'b'); figure(gcf)
xlabel('x_1')
ylabel('x_2')
axis tight equal;

hold on
for x20 = -3:0.5:3
     for x10 = -3:0.5:3
    [ts,xs] = ode45(F1,[0,50],[x20;x20]);
    plot(xs(:,1),xs(:,2))
    [ts,xs] = ode45(F2,[0,50],[x20;x20]);
    plot(xs(:,1),xs(:,2))
    axis( [-3 3 -3 3] )

     end

end
hold off
