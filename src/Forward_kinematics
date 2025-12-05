function [x1,y1,x2,y2] = get_forward_kinematics(theta1,theta2,l1,l2)

x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);

x2 = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
y2 = l1 * sin(theta1) + l2 * sin(theta1 + theta2);

end
