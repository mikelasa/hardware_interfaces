clear all;
close all;
clc;

% ---- Load the file ----
filename = 'impedance_controller.log'; % change to your file name
data = readmatrix(filename);        % assumes whitespace-separated txt file

% ---- Extract columns ----
t                   = data(:,1);          % timestamp (de cada ciclo)
pose                = data(:,2:8);        % pose de referencia SE3_WTref
pose_ref            = data(:,9:15);        % pose actual SE3_WT
pose_error          = data(:,16:21);       % pose comandada (step) SE3_WT_cmd
dq                  = data(:,22:28);      % wrench medida wrench_T_fb
wrench              = data(:,29:34);      % wrench comandada wrench_Tr_All
tau_task            = data(:,35:41);      % velocidad espacial referrncia v_spatial_WT
tau_d               = data(:,42:48);      % body velocity referencia  v_body_WT_ref

t_abs = cumsum(t);

% ---- Plot Pose ----
figure;
pose_labels = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3,1,i)
    plot(t_abs, pose(:,i), 'LineWidth', 1.2)
    hold on
    plot(t_abs, pose_ref(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['pose [' pose_labels{i} ']'])
    legend('pose\_ref', 'pose')
    if i==1
        title('Pose')
    end
    grid on
    
end

% ---- Plot velocity ----
figure;
velocity_labels = {'dq 1', 'dq 2', 'dq 3', 'dq 4', 'dq 5', 'dq 6'};
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, dq(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(velocity_labels{i})
    legend('velocity')
    if i==1
        title('joint velocity')
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
    xlabel('Time [s]')
    ylabel(wrench_labels{i})
    legend('wrench')
    if i==1
        title('Wrench')
    end
    grid on
    hold off
end

% ---- Plot torques ----
figure;
torques_labels = {'tau 1', 'tau 2', 'tau 3', 'tau 4', 'tau 5', 'tau 6'};
for i = 1:6
    subplot(6,1,i)
    plot(t_abs, tau_task(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(torques_labels{i})
    legend('torques')
    if i==1
        title('task torques')
    end
    grid on
    hold off
end

