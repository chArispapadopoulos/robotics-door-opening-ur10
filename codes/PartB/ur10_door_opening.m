clc; clear; close all;

load('handle_traj.mat');  % t, T_E_log
dt = t(2) - t(1);
n = length(t);

q0 = [-1.7752, -1.1823, 0.9674, 0.2149, 1.3664, 1.5708];
q = q0;
qm = zeros(n, 6);
qm(1,:) = q0;

robot = ur10robot();

l = 1.0; h = 0.7; lo = 0.1;
R_dh_align = [0 1 0; -1 0 0; 0 0 1];
T_EH = transl(0.1, 0.1, 0) * [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
T_align = trotz(-pi/2) * troty(pi);

theta_h1 = deg2rad(-45);
theta_d1 = deg2rad(-30);
t1 = 1.5; t2 = 4.0;

figure; set(gcf, 'Color', 'w');
for i = 1:n
    if t(i) <= t1
        theta_h = quinticPoly(0, t1, 0, theta_h1, t(i));
        theta_d = 0;
    elseif t(i) <= t2
        theta_h = theta_h1;
        theta_d = quinticPoly(t1, t2, 0, theta_d1, t(i));
    else
        theta_h = quinticPoly(t2, t(end), theta_h1, 0, t(i));
        theta_d = theta_d1;
    end

    R_d = trotz(theta_d);
    T_d = transl([0 2 0]) * R_d;

    T_h_rel = transl([l 0 h]) * r2t(R_dh_align) * trotx(theta_h) * transl([lo 0 0]);
    T_H = T_d * T_h_rel;

    T_des = T_E_log(:,:,i);

    for j = 1:5
        T_curr = robot.fkine(q).T;
        xi = tr2delta(T_curr, T_des);
        if norm(xi) < 1e-3
            break
        end
         xi = tr2delta(T_curr, T_des)/dt;
        J = robot.jacobe(q);
        q_dot = pinv(J) * xi;
        q = q + q_dot' * dt;
    end

    qm(i,:) = q;

    if mod(i,3)==0 || i==n
        clf;
        hold on; axis equal; grid on;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        xlim([-1 3]); ylim([-1 3]); zlim([0 2]);
        view(40, 30);

    
        door_w = l; door_h = 1.9;
        door = [0 0 0; door_w 0 0; door_w 0 door_h; 0 0 door_h]';
        door_global = T_d(1:3,1:3) * door + T_d(1:3,4);
        fill3(door_global(1,:), door_global(2,:), door_global(3,:), ...
            [0.8 0.8 1.0], 'FaceAlpha', 0.7, 'EdgeColor', 'k');

        trplot(T_H, 'frame', 'H', 'color', 'r', 'length', 0.2);
        trplot(T_des, 'frame', 'E', 'color', 'g', 'length', 0.2);
        robot.plot(q, 'workspace', [-1 3 -1 3 0 2], 'delay', 0);

        title(sprintf('t = %.2f s | θ_H = %.1f° | θ_D = %.1f°', ...
            t(i), rad2deg(theta_h), rad2deg(theta_d)));
        drawnow;
    end
end


qdot_log = diff(qm) / dt;
qdot_log(end+1,:) = qdot_log(end,:);  % αντιγραφή τελευταίας γραμμής για ίδιο μέγεθος με t

figure;
plot(t, qm);
xlabel('Χρόνος [s]');
ylabel('Θέση Αρθρώσεων [rad]');
title('Θέσεις Αρθρώσεων UR10');
legend('q_1','q_2','q_3','q_4','q_5','q_6');
grid on;

figure;
plot(t, qdot_log);
xlabel('Χρόνος [s]');
ylabel('Ταχύτητα Αρθρώσεων [rad/s]');
title('Ταχύτητες Αρθρώσεων UR10');
legend('q̇_1','q̇_2','q̇_3','q̇_4','q̇_5','q̇_6');
grid on;

function ur10 = ur10robot()
    ur10 = mdl_ur10();
    ur10.base.t = [1; 1; 0];
end

function r = mdl_ur10()
    a = [0, -0.612, -0.5723, 0, 0, 0]';
    d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]';
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]';
    DH = [zeros(6,1), d, a, alpha];
    robot = SerialLink(DH, 'name', 'UR10');
    mass = [7.1, 12.7, 4.27, 2.0, 2.0, 0.365];
    com = [
        0.021, 0, 0.027;
        0.38, 0, 0.158;
        0.24, 0, 0.068;
        0.0, 0.007, 0.018;
        0.0, 0.007, 0.018;
        0, 0, -0.026];
    for i = 1:6
        robot.links(i).m = mass(i);
        robot.links(i).r = com(i,:);
    end
    r = robot;
end

function q = quinticPoly(t0, tf, q0, qf, t)
    T = tf - t0;
    tau = (t - t0) / T;
    tau = max(0, min(1, tau));
    q = q0 + 10*(qf - q0)*tau.^3 - 15*(qf - q0)*tau.^4 + 6*(qf - q0)*tau.^5;
end