clc
clear
close all

%% Forward

syms L1_ L2_
syms thetaA thetaB

alpha_f1 = 0.5 * (pi - thetaA - thetaB);
gamma_f1 = asin(L1_*sin(alpha_f1)/L2_);
phi_f1 = pi - alpha_f1 - gamma_f1;

theta_f1 = thetaA + alpha_f1;
R_f1 = L2_*sin(phi_f1)/sin(alpha_f1);

X_f1 = R_f1*cos(theta_f1);
Y_f1 = R_f1*sin(theta_f1);

P_f1 = [X_f1; Y_f1];

J_f1 = jacobian(P_f1, [thetaA; thetaB]);

forward_jacobian1 = simplify(J_f1)

getForwardJacobian1 = matlabFunction(forward_jacobian1);

%%

alpha_f2 = 0.5 * (pi + thetaA + thetaB);
gamma_f2 = asin(L1_*sin(alpha_f2)/L2_);
phi_f2 = pi - alpha_f2 - gamma_f2;

theta_f2 = thetaA - alpha_f2;
R_f2 = L2_*sin(phi_f2)/sin(alpha_f2);

X_f2 = R_f2*cos(theta_f2);
Y_f2 = R_f2*sin(theta_f2);

P_f2 = [X_f2; Y_f2];

J_f2 = jacobian(P_f2, [thetaA; thetaB]);

forward_jacobian2 = simplify(J_f2)

getForwardJacobian2 = matlabFunction(forward_jacobian2);


%%

L1 = 0.05;
L2 = 0.15;
thA = -0.78;
thB = -0.78;

J1_00 = getForwardJacobian1(L1, L2, thA, thB)
J2_00 = getForwardJacobian2(L1, L2, thA, thB)

F = [0; -5];

T1 = J1_00' * F
T2 = J2_00' * F