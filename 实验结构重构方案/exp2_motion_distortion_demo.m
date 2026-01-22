%% EXP2_MOTION_DISTORTION_DEMO 运动畸变验证
% 作者: Komorebi

clear; clc; close all;

fprintf('===== 实验2: 运动畸变校正 =====\n\n');

%% 加载数据
script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(genpath(project_root));

data_file = fullfile(project_root, 'data', 'airplane', 'airplane_0010.txt');
if ~exist(data_file, 'file')
    error('找不到点云文件: %s', data_file);
end

raw_data = load(data_file);
original_pts = raw_data(:, 1:3);
N = size(original_pts, 1);
fprintf('点云: %d 点\n', N);

%% 定义运动畸变
fprintf('\n>>> 运动畸变 (扫描过程中物体移动)\n');

[~, scan_order] = sort(original_pts(:, 2));
t = (1:N)' / N;

motion_velocity = [0.5, 0.3, 0.1];
motion_rotation_rate = deg2rad(20);

fprintf('  平移速度: [%.2f, %.2f, %.2f]\n', motion_velocity);
fprintf('  旋转速率: %.1f°/周期\n', rad2deg(motion_rotation_rate));

distorted_pts = original_pts;
for i = 1:N
    idx = scan_order(i);
    ti = t(i);
    dx = motion_velocity(1) * ti;
    dy = motion_velocity(2) * ti;
    dz = motion_velocity(3) * ti;
    theta = motion_rotation_rate * ti;
    R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    distorted_pts(idx, :) = (R * original_pts(idx, :)')' + [dx, dy, dz];
end

true_rotation = [0, 0, motion_rotation_rate * 0.5];
true_scale = [1, 1, 1];

% 添加噪声
noise_level = 0.001;
distorted_pts = distorted_pts + noise_level * randn(N, 3);

%% 选择控制点
n_ctrl = 10;
ctrl_step = floor(N / n_ctrl);
ctrl_indices = (1:ctrl_step:N);
ctrl_indices = ctrl_indices(1:n_ctrl);
src_ctrl = distorted_pts(ctrl_indices, :);
dst_ctrl = original_pts(ctrl_indices, :);
fprintf('控制点: %d\n', n_ctrl);

%% 执行畸变消除
fprintf('\n>>> 执行校正\n');

options = struct('validate', true, 'visualize', false);
options.thresholds.scale_tolerance = 0.50;
options.thresholds.shear_tolerance = deg2rad(20);

[corrected_pts, ~, stats] = affine12_main(distorted_pts, src_ctrl, dst_ctrl, options);

%% 可视化
view_angle = [45, 25];

figure('Name', '运动畸变校正', 'Position', [50, 100, 1600, 500]);

subplot(1, 3, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, original_pts(:,3), 'filled');
title('原始'); axis equal; grid on; colorbar; view(view_angle);

subplot(1, 3, 2);
scatter3(distorted_pts(:,1), distorted_pts(:,2), distorted_pts(:,3), 1, 'r', 'filled');
title('运动畸变'); axis equal; grid on; view(view_angle);

subplot(1, 3, 3);
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, 'g', 'filled');
title('校正后'); axis equal; grid on; view(view_angle);

figure('Name', '叠加对比', 'Position', [100, 100, 1200, 500]);

subplot(1, 2, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, 'b', 'filled'); hold on;
scatter3(distorted_pts(:,1), distorted_pts(:,2), distorted_pts(:,3), 1, 'r', 'filled'); hold off;
title('原始(蓝) vs 畸变(红)'); axis equal; grid on; legend('原始', '畸变'); view(view_angle);

subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, 'b', 'filled'); hold on;
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, 'g', 'filled'); hold off;
title('原始(蓝) vs 校正(绿)'); axis equal; grid on; legend('原始', '校正后'); view(view_angle);

%% 误差统计
errors = corrected_pts - original_pts;
error_mag = sqrt(sum(errors.^2, 2));
rmse_recovery = sqrt(mean(error_mag.^2));

distortion_errors = distorted_pts - original_pts;
rmse_distortion = sqrt(mean(sum(distortion_errors.^2, 2)));

fprintf('\n===== 结果 =====\n');
fprintf('畸变RMSE: %.6f\n', rmse_distortion);
fprintf('校正RMSE: %.6f\n', rmse_recovery);
fprintf('改进倍数: %.1fx\n', rmse_distortion / rmse_recovery);
fprintf('注: 运动畸变为非全局变换，仿射变换只能部分校正\n');
fprintf('================\n');

figure('Name', '误差分析', 'Position', [150, 100, 1000, 400]);
subplot(1, 2, 1);
histogram(error_mag, 50, 'FaceColor', [0.2, 0.6, 0.9]);
title('误差分布'); xlabel('误差'); ylabel('频数'); grid on;

subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, error_mag, 'filled');
title('误差空间分布'); colorbar; axis equal; grid on; view(view_angle);
