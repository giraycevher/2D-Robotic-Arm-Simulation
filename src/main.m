clc;
clear all;
clf;

link_length_m = [0.1 0.1];
link_length_mm = link_length_m * 1e3;
g = 9.81;
m1 = 0.1;
m2 = 0.1;
workspace_traj = [199.999 0; 100 100; 0 199.999; -100 100; -199.999 0];
solution_selection = 1;

Q_total = [];
Q_dot_total = [];
Q_ddot_total = [];
Torque_total = [];
t_total = [];
T_sim = 5;
dt = 0.001;

End_Effector_Path = [];
mid_link_Path = [];

for j = 2:size(workspace_traj, 1)
    [qs1_r, qs2_r] = get_inverse_kinematics(workspace_traj(j-1, 1), workspace_traj(j-1, 2), link_length_mm, solution_selection);
    [qf1_r, qf2_r] = get_inverse_kinematics(workspace_traj(j, 1), workspace_traj(j, 2), link_length_mm, solution_selection);
    
    qs1 = rad2deg(qs1_r);
    qs2 = rad2deg(qs2_r);
    qf1 = rad2deg(qf1_r);
    qf2 = rad2deg(qf2_r);
    
    [q1, v1, a1, t1] = fifth_order_trajectory(qs1, qf1, T_sim, dt);
    [q2, v2, a2, t2] = fifth_order_trajectory(qs2, qf2, T_sim, dt);
    
    q1 = q1(:);
    v1 = v1(:);
    a1 = a1(:);
    q2 = q2(:);
    v2 = v2(:);
    a2 = a2(:);
    
    Ttotal_1 = t1(end);
    Ttotal_2 = t2(end);
    T_max = max(Ttotal_1, Ttotal_2);
    
    t_seg = (0:dt:T_max);
    numstep = length(t_seg);
    
    q_seg_deg = [q1, q2];
    q_dot_seg_deg = [v1, v2];
    q_ddot_seg_deg = [a1, a2];
    
    q_seg = deg2rad(q_seg_deg);
    q_dot_seg = deg2rad(q_dot_seg_deg);
    q_ddot_seg = deg2rad(q_ddot_seg_deg);
    
    torque_seg = zeros(size(q1, 1), 2);
    
    for i = 1:numstep
        theta1 = q_seg(i, 1);
        theta2 = q_seg(i, 2);
        theta1_dot = q_dot_seg(i, 1);
        theta2_dot = q_dot_seg(i, 2);
        q_ddot_vec_rad = q_ddot_seg(i, :)';
        
        M = Mass_matris(theta2, link_length_m(1), link_length_m(2), m1, m2);
        C = Cent_matris(theta2, theta1_dot, theta2_dot, link_length_m(1), link_length_m(2), m2);
        G = Gravity_matris(theta1, theta2, link_length_m(1), link_length_m(2), m1, m2);
        
        tau = (M * q_ddot_vec_rad) + C + G;
        torque_seg(i, :) = tau';
    end
    
    current_time_offset = 0;
    if ~isempty(t_total)
        current_time_offset = t_total(end);
    end
    
    t_total = [t_total; t_seg(:) + current_time_offset];
    Q_total = [Q_total; q_seg];
    Q_dot_total = [Q_dot_total; q_dot_seg];
    Q_ddot_total = [Q_ddot_total; q_ddot_seg];
    Torque_total = [Torque_total; torque_seg];
end

simulation_time = T_max;

figure;
subplot(2, 1, 1);
plot(t_total, Torque_total(:, 1), 'r', 'LineWidth', 1.5); 
hold on;
plot(t_total, Torque_total(:, 2), 'b', 'LineWidth', 1.5);
title('Joint Torque (Inverse Dynamics)');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('\tau_1 (Joint 1)', '\tau_2 (Joint 2)');
grid on;

subplot(2, 1, 2);
plot(t_total, Q_total(:, 1), 'r', 'LineWidth', 1.5); 
hold on;
plot(t_total, Q_total(:, 2), 'b', 'LineWidth', 1.5);
title('Joint Position (Rad)');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend('q_1', 'q_2');
grid on;

skip_rate = 180;
home_pose = [0 0];

figure('units', 'normalized', 'outerposition', [0 0 1 1], 'color', 'w');

for i = 1:skip_rate:size(Q_total, 1)
    theta1_rad = Q_total(i, 1);
    theta2_rad = Q_total(i, 2);
    
    [x1, y1, x2, y2] = get_forward_kinematics(theta1_rad, theta2_rad, link_length_mm(1), link_length_mm(2));
    
    End_Effector_Path = [End_Effector_Path; x2, y2];
    mid_link_Path = [mid_link_Path; x1, y1];
    
    clf;
    
    plot([home_pose(1) x1], [home_pose(2) y1], "cyan", 'LineWidth', 4);
    hold on;
    plot([x1 x2], [y1 y2], "cyan", 'LineWidth', 4);
    
    scatter(x1, y1, 100, "r", "filled");
    scatter(x2, y2, 100, "b", "filled");
    scatter(home_pose(1), home_pose(2), 100, "r", "filled");
    
    if size(End_Effector_Path, 1) > 1
        plot(End_Effector_Path(:, 1), End_Effector_Path(:, 2), 'g--', 'LineWidth', 1);
    end
    
    if size(mid_link_Path, 1) > 1
        plot(mid_link_Path(:, 1), mid_link_Path(:, 2), 'g--', 'LineWidth', 1);
    end
    
    axis equal;
    axis([-300 300 -300 300]);
    xline(0);
    yline(0);
    xlabel("X [mm]");
    ylabel("Y [mm]");
    title(sprintf("2 DOF Robot Sim | Time: %.2f s", t_total(i)));
    drawnow;
end
