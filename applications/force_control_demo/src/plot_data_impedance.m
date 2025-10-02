%% 
clear all;
close all;
clc;

% ---- Load the file ----
filename = 'impedance_controller.log'; % change to your file name
data = readmatrix(filename);        % assumes whitespace-separated txt file

% ---- Extract columns ----
t                   = data(:,1);           % timestamp (de cada ciclo)
pose                = data(:,2:8);         % pose actual (quats)
pose_ref            = data(:,9:15);        % pose de referencia 
q                   = data(:,16:22);       % joints posicion
dq                  = data(:,23:29);       % wrench medida wrench_T_fb
wrench              = data(:,30:35);       % wrench comandada wrench_Tr_All
tau_task            = data(:,36:41);       % task torques
tau_nullspace       = data(:,42:48);       % nullspace torques
tau_ext             = data(:,42:48);       % external torques
tau_d               = data(:,42:48);       % desired torques

t_abs = cumsum(t);
%from quats to axis angle
% Convert quaternion to axis-angle representation
pose_axis_angle = zeros(size(pose, 1), 4);
pose_ref_axis_angle = zeros(size(pose, 1), 4);

for i = 1:size(pose, 1)
    pose_axis_angle(i, :) = quat2axang(pose(i, 4:7));
    pose_ref_axis_angle(i, :) = quat2axang(pose_ref(i, 4:7));
end

% ---- Plot Pose positions----
figure;
pose_labels = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3,1,i)
    plot(t_abs, pose(:,i), 'LineWidth', 1.2)
    hold on
    plot(t_abs, pose_ref(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['pose [' pose_labels{i} ']'])
    legend('pose', 'pose\_ref')
    if i==1
        title('Pose')
    end
    grid on
    
end
% 
% % ---- Plot Pose orientations----
% figure;
% pose_labels = {'w', 'x', 'y', 'z'};
% for i = 1:4
%     subplot(4,1,i)
%     plot(t_abs, pose(:,i+3), 'LineWidth', 1.2)
%     hold on
%     plot(t_abs, pose_ref(:,i+3), 'LineWidth', 1.2)
%     plot(t_abs, pose_error(:,i+3), 'LineWidth', 1.2)
%     xlabel('Time [s]')
%     ylabel(['pose [' pose_labels{i} ']'])
%     legend('pose', 'pose\_ref', 'pose\_error')
%     if i==1
%         title('quat error')
%     end
%     grid on
% 
% end

% ---- Plot Pose orientations axis angle----
figure;
pose_labels = {'a', 'b', 'c'};
for i = 1:3
    subplot(3,1,i)
    plot(t_abs, pose_axis_angle(:,i), 'LineWidth', 1.2)
    hold on
    plot(t_abs, pose_ref_axis_angle(:,i), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['pose [' pose_labels{i} ']'])
    legend('pose', 'pose\_ref')
    if i==1
        title('axis angle error')
    end
    grid on
    
end

% ---- Plot wrench----
figure;
wrench_labels = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
for j = 1:6
    subplot(6,1,j)
    plot(t_abs, wrench(:,j), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['wrench [' wrench_labels{j} ']'])
    if i==1
        title('Wrench filtered')
    end
    grid on
end

%---- Plot torques----
figure;
torque_labels = {'t1', 't2', 't3', 't4', 't5', 't6'};
for k = 1:6
    subplot(6,1,k)
    plot(t_abs, tau_d(:,k), 'LineWidth', 1.2)
    hold on
    plot(t_abs, tau_task(:,k), 'LineWidth', 1.2)
    plot(t_abs, tau_nullspace(:,k), 'LineWidth', 1.2)
    plot(t_abs, tau_ext(:,k), 'LineWidth', 1.2)
    xlabel('Time [s]')
    ylabel(['Torque [' torque_labels{k} ']'])
    if k==1
        title('Torques ')
    end
    grid on
end