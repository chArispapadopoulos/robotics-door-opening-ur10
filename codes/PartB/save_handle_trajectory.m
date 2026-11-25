clc; clear; close all;

T_total = 5;
dt = 0.05;
t = 0:dt:T_total;

theta_h1 = deg2rad(-45);  
theta_d1 = deg2rad(-30);  
t1 = 1.5; t2 = 4.0;

theta_h = zeros(size(t));
theta_d = zeros(size(t));

for i = 1:length(t)
    if t(i) <= t1
        theta_h(i) = quinticPoly(0, t1, 0, theta_h1, t(i));
        theta_d(i) = 0;
    elseif t(i) <= t2
        theta_h(i) = theta_h1;
        theta_d(i) = quinticPoly(t1, t2, 0, theta_d1, t(i));
    else
        theta_h(i) = quinticPoly(t2, T_total, theta_h1, 0, t(i));
        theta_d(i) = theta_d1;
    end
end

l = 1.0; h = 0.7; lo = 0.1;

T_EH = transl(0.1, 0.1, 0) * [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
T_align = trotz(-pi/2) * troty(pi);

q_E = zeros(4, length(t));
p_E = zeros(3, length(t));
T_E_log = zeros(4,4,length(t));

figure;
set(gcf, 'Color', 'w');
for i = 1:length(t)

    R_d = trotz(theta_d(i));
    T_d = transl([0 2 0]) * R_d;

    R_dh_align = [0 1 0; -1 0 0; 0 0 1];
    T_h_rel = transl([l 0 h]) * r2t(R_dh_align) * trotx(theta_h(i)) * transl([lo 0 0]);
    T_H = T_d * T_h_rel;

    T_E = T_H * T_EH * T_align;
    T_E_log(:,:,i) = T_E;

    R_E = T_E(1:3,1:3);
    p_E(:,i) = T_E(1:3,4);
    q_E(:,i) = UnitQuaternion(R_E).double';

    clf;
    hold on; axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-1.5 1.5]); ylim([-0.5 2.5]); zlim([0 2]);
    view(40, 30);

    trplot(eye(4), 'frame', '0', 'length', 0.2);
    trplot(T_d, 'frame', 'D', 'color', 'b', 'length', 0.2);
    trplot(T_H, 'frame', 'H', 'color', 'r', 'length', 0.25, 'thick', 2);
    trplot(T_E, 'frame', 'E', 'color', 'g', 'length', 0.25, 'thick', 2);

    door_w = l; door_h = 1.9;
    door = [0 0 0; door_w 0 0; door_w 0 door_h; 0 0 door_h]';
    door_global = T_d(1:3,1:3) * door + T_d(1:3,4);
    fill3(door_global(1,:), door_global(2,:), door_global(3,:), ...
        [0.7 0.7 0.9], 'FaceAlpha', 0.8, 'EdgeColor', 'k');

    title(sprintf('t = %.2f s | θ_H = %.1f° | θ_D = %.1f°', ...
        t(i), rad2deg(theta_h(i)), rad2deg(theta_d(i))));
    pause(0.0001);
end

save('handle_traj.mat', 't', 'T_E_log');


figure;
plot(t, q_E); legend('q0','q1','q2','q3');
title('Προσανατολισμός πλαισίου {E}');
xlabel('Time [s]'); ylabel('Quaternion');

function q = quinticPoly(t0, tf, q0, qf, t)
    T = tf - t0;
    tau = (t - t0) / T;
    tau = max(0, min(1, tau));
    q = q0 + 10*(qf - q0)*tau.^3 - 15*(qf - q0)*tau.^4 + 6*(qf - q0)*tau.^5;
end
