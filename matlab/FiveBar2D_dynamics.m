clc
clear
close all

%% Forward

syms L1_M L2_M
syms thetaA thetaB

alpha_f = 0.5 * (pi + thetaA + thetaB);
gamma_f = asin(L1_M*sin(alpha_f)/L2_M);
phi_f = pi - alpha_f - gamma_f;

theta_f = thetaA - alpha_f;
R_f = L2_M*sin(phi_f)/sin(alpha_f);

X_f = R_f*cos(theta_f);
Y_f = R_f*sin(theta_f);

P_f = [X_f; Y_f];

J_f = jacobian(P_f, [thetaA; thetaB]);

forward_jacobian = simplify(J_f)

getForwardJacobian = matlabFunction(forward_jacobian);


%% Test

L1 = 0.05;
L2 = 0.15;

thA = 0;
thB = 0;

Jf_00 = getForwardJacobian(L1, L2, thA, thB)

Ff = [0; -5];

Tf = Jf_00' * Ff

%% INVERSE

inverse_jacobian = inv(forward_jacobian)

getInverseJacobian = matlabFunction(inverse_jacobian);

%% Test

Ji_00 = getInverseJacobian(L1, L2, thA, thB)


