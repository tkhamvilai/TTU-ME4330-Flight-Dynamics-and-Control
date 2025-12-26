clear; clc; close all

% There are three equilibrium points
% (0,2*sqrt(2)), (0,-2*sqrt(2)), and (-2,0)

% f1 = @(x1,x2) x1.^3 - x2.^2 + 8;
% f2 = @(x1,x2) x1.*x2;

% The equilibrium point is (x1,x2) = (0,0)
f1 = @(x1,x2) -x1./(1+x1.^2).^2 + x2;
f2 = @(x1,x2) -(x1+x2)./(1+x1.^2).^2;
[x1,x2] = meshgrid(-10:0.1:10, -10:0.1:10);
quiver(x1,x2,f1(x1,x2),f2(x1,x2),6)
xlabel('x1')
ylabel('x2')

A = @(x1e, x2e) [3*x1e^2 -2*x2e; x2e x1e];
eig(A(0,2*sqrt(2)))
eig(A(0,-2*sqrt(2)))
eig(A(-2,0))