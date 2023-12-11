clc
clear
close all
%%

syms L1 L2
syms A B C

alpha = 0.5 * (pi - A - B);
gamma = asin(L1*sin(alpha)/L2);
phi = pi - alpha - gamma;

theta = A + alpha;
R = L2*sin(phi)/sin(alpha);

X = R*cos(theta);
Y = 0;
Z = R*sin(theta);

P = [X; Y; Z];

Rot = [1, 0, 0; 0, cos(C), -sin(C); 0, sin(C), cos(C)];

Prot = Rot*P;

J = jacobian(P, [A; B; C]);