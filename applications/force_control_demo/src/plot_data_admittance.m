clear all;
close all;
clc;

% ---- Load the file ----
filename = 'impedance_controller.log'; % change to your file name
data = readmatrix(filename);        % assumes whitespace-separated txt file

% ---- Extract columns ----
t                   = data(:,1);          % timestamp (de cada ciclo)
pose_ref            = data(:,2:4);        % pose de referencia SE3_WTref
pose                = data(:,5:7);        % pose actual SE3_WT
pose_cmd            = data(:,8:10);       % pose comandada (step) SE3_WT_cmd
wrench              = data(:,11:16);      % wrench medida wrench_T_fb
wrench_cmd          = data(:,17:22);      % wrench comandada wrench_Tr_All
v_spatial_WT        = data(:,23:28);      % velocidad espacial referrncia v_spatial_WT
v_body_WT_ref       = data(:,29:34);      % body velocity referencia  v_body_WT_ref

wrench_Tr_spring    = data(:,35:40);
wrench_Tr_Err       = data(:,41:46);
wrench_Tr_PID       = data(:,47:52);
wrench_Tr_damping   = data(:,53:58);

t_abs = cumsum(t);

% ---- Plot Pose ----
figure;
pose_labels = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3,1,i)
    plot(t_abs, pose_ref(:,i), 'LineWidth', 1.2)
    hold on
    plot(t_abs, pose(:,i), 'LineWidth', 1.2)
    plot(t_abs, pose_cmd(:,i), '--', 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['pose [' pose_labels{i} ']'])
    legend('pose\_ref', 'pose', 'pose\_cmd')
    if i==1
        title('Pose')
    end
    grid on
    hold off
end

% ---- Plot Wrench ----
figure;
wrench_labels = {'force X','force Y','force Z','torque X','torque Y','torque Z'};
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, wrench(:,i), 'LineWidth', 1.2)
    hold on
    plot(t_abs, wrench_cmd(:,i), '--', 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(wrench_labels{i})
    legend('wrench', 'wrench\_Tr\_All')
    if i==1
        title('Wrench')
    end
    grid on
    hold off
end

% ---- Plot Body Velocity ----
figure;
body_velocity_labels = {'v_body_x', 'v_body_y', 'v_body_z'};
for i = 1:3
    subplot(3,1,i)
    plot(t_abs, v_body_WT_ref(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(body_velocity_labels{i})
    legend('v\_body\_WT\_ref')
    if i==1
        title('Body Velocity')
    end
    grid on
    hold off
end

% ---- Plot Spatial ----
figure;
velocity_labels = {'v_x', 'v_y', 'v_z', 'omega_x', 'omega_y', 'omega_z'};
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, v_spatial_WT(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(velocity_labels{i})
    legend('v\_spatial\_WT')
    if i==1
        title('Spatial velocity (force convertion')
    end
    grid on
    hold off
end

% ---- plot spring wrench
% Plot Spring Wrench
figure;
spring_wrench_labels = {'spring_force_x', 'spring_force_y', 'spring_force_z', 'spring_torque_x', 'spring_torque_y', 'spring_torque_z'};
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, wrench_Tr_spring(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(spring_wrench_labels{i})
    legend('spring\_wrench')
    if i==1
        title('Spring Wrench')
    end
    grid on
    hold off
end

% ---- Plot Wrench Tracking Error ----
figure;
wrench_error_labels = {'error_force_x', 'error_force_y', 'error_force_z', 'error_torque_x', 'error_torque_y', 'error_torque_z'};
% Calculate and plot Wrench Tracking Error
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, wrench_Tr_Err(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(wrench_error_labels{i})
    legend('wrench\_error')
    if i==1
        title('Wrench Tracking Error')
    end
    grid on
    hold off
end

% ---- Plot Wrench PID ----
figure;
wrench_pid_labels = {'pid_force_x', 'pid_force_y', 'pid_force_z', 'pid_torque_x', 'pid_torque_y', 'pid_torque_z'};
% Calculate and plot Wrench PID
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, wrench_Tr_PID(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(wrench_pid_labels{i})
    legend('wrench\_PID')
    if i==1
        title('Wrench PID')
    end
    grid on
    hold off
end

% ---- Plot Wrench Damping ----
figure;
wrench_damping_labels = {'damping_force_x', 'damping_force_y', 'damping_force_z', 'damping_torque_x', 'damping_torque_y', 'damping_torque_z'};
% Calculate and plot Wrench Damping
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, wrench_Tr_damping(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(wrench_damping_labels{i})
    legend('wrench\_damping')
    if i==1
        title('Wrench Damping')
    end
    grid on
    hold off
end



