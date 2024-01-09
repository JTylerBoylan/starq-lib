clc
clear
close all
%%

syms L1_ L2_
syms thetaA thetaB

alpha = 0.5 * (pi - thetaA - thetaB);
gamma = asin(L1_*sin(alpha)/L2_);
phi = pi - alpha - gamma;

theta = thetaA + alpha;
R = L2_*sin(phi)/sin(alpha);

X = R*cos(theta);
Z = R*sin(theta);

P = [X; Z];

%Rot = [1, 0, 0; 0, cos(C), -sin(C); 0, sin(C), cos(C)];

%Prot = Rot*P;

J = jacobian(P, [thetaA; thetaB]);

jacobian = simplify(J)