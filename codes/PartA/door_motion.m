    clc; clear; close all;
    
    T_total = 5;
    dt = 0.05;
    t = 0:dt:T_total;
     theta_h1 = deg2rad(-45); 
    theta_d1 = deg2rad(-30);
    t1 = 1.5;
    t2 = 4.0;
    
    
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
    
    
    l = 1.0;
    h = 0.7;
    lo = 0.1;
    
    q_h = zeros(4, length(t));
    
    figure;
    set(gcf, 'Color', 'w');
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-1.5 1.5]); ylim([-0.5 2.5]); zlim([0 2]);
    
    for i = 1:length(t)
        R_d = trotz(theta_d(i));
        T_d = transl([0 2 0]) * R_d;
    
        
        R_dh_align = [ 0  1 0;
                      -1  0 0;
                       0  0 1 ];
    
        
        T_h_rel = transl([l 0 h]) * r2t(R_dh_align) * trotx(theta_h(i)) * transl([lo 0 0]);
        T_h = T_d * T_h_rel;
    
        q_h(:,i) = UnitQuaternion(t2r(T_h)).double';
    
        clf;
        hold on;
    
        
        floor_x = [-2 2 2 -2];
        floor_y = [-1 -1 3 3];
        fill3(floor_x, floor_y, [0 0 0 0], [0.9 0.9 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
    
        trplot(eye(4), 'frame', '0', 'rgb', 'length', 0.2);
        trplot(T_d, 'frame', 'D', 'color', 'b', 'length', 0.2);
        trplot(T_h, 'frame', 'H', 'color', 'r', 'length', 0.2);
    
        
        door_width = l;
        door_height = 1.9;
        local_door = [0 0 0;
                      door_width 0 0;
                      door_width 0 door_height;
                      0 0 door_height]';
        global_door = T_d(1:3,1:3) * local_door + T_d(1:3,4);
        fill3(global_door(1,:), global_door(2,:), global_door(3,:), ...
            [0.7 0.7 0.9], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
    
        
        p0 = [0 2 0];
        pd = T_d(1:3,1:3) * [l; 0; 0] + T_d(1:3,4);
        plot3([p0(1) pd(1)], [p0(2) pd(2)], [0 0], 'k:', 'LineWidth', 1.5);
    
       
        plot3([0 0], [0 2], [0 0], 'g--', 'LineWidth', 1.5);
        text(0, 1, 0.05, '2m', 'Color', 'g', 'FontSize', 10);
    
        handle_pos = T_d(1:3,1:3)*[l; 0; h] + T_d(1:3,4);
        plot3([handle_pos(1) handle_pos(1)], [handle_pos(2) handle_pos(2)], ...
            [0 handle_pos(3)], 'm--', 'LineWidth', 1.5);
        text(handle_pos(1), handle_pos(2), handle_pos(3)/2, '0.7m', ...
            'Color', 'm', 'FontSize', 10);
    
        title(sprintf('t = %.2f s | θ_πόμολου = %.1f° | θ_πόρτας = %.1f°', ...
            t(i), rad2deg(theta_h(i)), rad2deg(theta_d(i))), 'FontSize', 12);
    
        view(40, 30);
        pause(0.01);
    end
    
    figure;
    plot(t, q_h);
    legend('q_0','q_1','q_2','q_3');
    xlabel('Time [s]');
    ylabel('Quaternion Component');
    title('Unit Quaternion Orientation of Handle');
    
    function q = quinticPoly(t0, tf, q0, qf, t)
        T = tf - t0;
        tau = (t - t0) / T;
        tau = max(0, min(1, tau));
        q = q0 + 10*(qf - q0)*tau.^3 - 15*(qf - q0)*tau.^4 + 6*(qf - q0)*tau.^5;
    end
    
    
    
    

    

