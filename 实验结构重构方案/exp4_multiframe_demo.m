%% EXP4_MULTIFRAME_DEMO 多帧拼接错误验证
% 作者: Komorebi

clear; clc; close all;

fprintf('===== 实验4: 多帧拼接错误校正 =====\n\n');

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

%% 定义多帧拼接错误
fprintf('\n>>> 多帧拼接错误 (ICP配准失败)\n');

distorted_pts = original_pts;
n_frames = 4;
z_edges = linspace(min(original_pts(:,3)), max(original_pts(:,3)), n_frames + 1);

rng(42);
for f = 1:n_frames
    frame_mask = (original_pts(:,3) >= z_edges(f)) & (original_pts(:,3) < z_edges(f+1));
    frame_translation = 0.1 * (rand(1, 3) - 0.5);
    frame_rotation = deg2rad(5) * (rand(1, 3) - 0.5);
    fprintf('  帧%d: T=[%.2f,%.2f,%.2f], R=[%.1f°,%.1f°,%.1f°]\n', f, frame_translation, rad2deg(frame_rotation));
    M_frame = affine12_build_matrix(frame_rotation, [1,1,1], [0,0,0], frame_translation);
    distorted_pts(frame_mask, :) = affine12_apply(original_pts(frame_mask, :), M_frame);
end

true_rotation = [0, 0, 0];
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

figure('Name', '多帧拼接校正', 'Position', [50, 100, 1600, 500]);

subplot(1, 3, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, original_pts(:,3), 'filled');
title('原始'); axis equal; grid on; colorbar; view(view_angle);

subplot(1, 3, 2);
scatter3(distorted_pts(:,1), distorted_pts(:,2), distorted_pts(:,3), 1, 'r', 'filled');
title('多帧错位'); axis equal; grid on; view(view_angle);

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
fprintf('注: 多帧拼接错误需要ICP配准，仿射变换效果有限\n');
fprintf('================\n');

figure('Name', '误差分析', 'Position', [150, 100, 1000, 400]);
subplot(1, 2, 1);
histogram(error_mag, 50, 'FaceColor', [0.2, 0.6, 0.9]);
title('误差分布'); xlabel('误差'); ylabel('频数'); grid on;

subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, error_mag, 'filled');
title('误差空间分布'); colorbar; axis equal; grid on; view(view_angle);
