function [C_matris]=Cent_matrix(theta2,theta1_dot,theta2_dot, L1,L2,m2)
omega1=theta1_dot;
omega2=theta2_dot;
s2=sin(theta2);
C1 = -m2 * L1 * L2 * s2 * (2*omega1*omega2 + omega2^2);
C2 = m2 * L1 * L2 * s2 * (omega1^2);
C_matris=[C1;C2];
end
