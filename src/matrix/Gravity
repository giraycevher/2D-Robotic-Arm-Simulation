function [G_matris]=Gravity_matrix(theta1_rad,theta2_rad,L1,L2,m1,m2)
l1=L1/2;
l2=L2/2;
g= 9.81;

c1=cos(theta1_rad);
c12=cos(theta1_rad+theta2_rad);

G1 = (m1*l1 + m2*L1)*g*c1 + m2*g*l2*c12;
G2 = m2*g*l2*c12;
G_matris=[G1;G2];
end
