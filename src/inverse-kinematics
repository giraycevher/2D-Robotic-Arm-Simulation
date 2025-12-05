function [theta1,theta2] = get_inverse_kinematics(target_x,target_y,link_length,solution_selection)

  if solution_selection == 1
    tmp = (target_x^2 + target_y^2 - link_length(1)^2 - link_length(2)^2) / (2*link_length(1)*link_length(2));
    theta2 = atan2(sqrt(1-tmp^2),tmp);
    theta1 = atan2(target_y,target_x) - atan2(sqrt(target_y^2 + target_x^2 -(link_length(2)*cos(theta2)+link_length(1))^2),(link_length(2)*cos(theta2)+link_length(1)));

  elseif solution_selection == 2
    tmp = (target_x^2 + target_y^2 - link_length(1)^2 - link_length(2)^2) / (2*link_length(1)*link_length(2));
    theta2 = atan2(-sqrt(1-tmp^2),tmp);
    theta1 = atan2(target_y,target_x) + atan2(sqrt(target_y^2 + target_x^2 -(link_length(2)*cos(theta2)+link_length(1))^2),(link_length(2)*cos(theta2)+link_length(1)));


  end

end
