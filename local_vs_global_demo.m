%% LOCAL_VS_GLOBAL_DEMO 分块仿射 vs 全局仿射对比演示
%
% 本脚本对比分块局部仿射变换与全局仿射变换在处理多帧拼接错误时的效果差异
%
% 作者: 自动生成
% 日期: 2026-01-19

clear; clc; close all;

fprintf('============================================================\n');
fprintf('   分块仿射 vs 全局仿射 对比演示\n');
fprintf('============================================================\n\n');

%% 1. 加载点云数据
fprintf('>>> 步骤 1: 加载点云数据\n');

data_file = fullfile(pwd, 'airplane', 'airplane_0010.txt');
if ~exist(data_file, 'file')
    % 如果没有飞机数据，尝试汽车数据
    data_file = fullfile(pwd, 'car', 'car_0001.txt');
end

if ~exist(data_file, 'file')
    error('找不到点云文件，请确保在正确的目录下运行。');
end

raw_data = load(data_file);
original_pts = raw_data(:, 1:3);
N = size(original_pts, 1);
fprintf('  加载点云: %d 点\n', N);

%% 2. 创建多帧拼接畸变
fprintf('\n>>> 步骤 2: 创建多帧拼接畸变\n');

distorted_pts = original_pts;

% 将点云分成若干"帧"（按Z坐标分层）
n_frames = 4;
z_min = min(original_pts(:, 3));
z_max = max(original_pts(:, 3));
z_edges = linspace(z_min, z_max, n_frames + 1);

% 每帧添加不同的刚体变换（模拟配准误差）
rng(42);  % 固定随机种子

fprintf('  创建 %d 帧拼接畸变：\n', n_frames);
for f = 1:n_frames
    frame_mask = (original_pts(:,3) >= z_edges(f)) & (original_pts(:,3) < z_edges(f+1));
    
    % 随机刚体变换（每帧不同）
    frame_translation = 0.15 * (rand(1, 3) - 0.5);  % 随机平移
    frame_rotation = deg2rad(8) * (rand(1, 3) - 0.5);  % 随机旋转
    
    fprintf('    帧 %d: 平移=[%.3f, %.3f, %.3f], 旋转=[%.2f°, %.2f°, %.2f°]\n', ...
        f, frame_translation, rad2deg(frame_rotation));
    
    M_frame = affine12_build_matrix(frame_rotation, [1,1,1], [0,0,0], frame_translation);
    distorted_pts(frame_mask, :) = affine12_apply(original_pts(frame_mask, :), M_frame);
end

% 添加噪声
noise_level = 0.001;
distorted_pts = distorted_pts + noise_level * randn(N, 3);

%% 3. 选择控制点（均匀分布在各帧中）
fprintf('\n>>> 步骤 3: 选择控制点\n');

n_ctrl_per_frame = 6;  % 每帧6个控制点（更多控制点提高估计精度）
ctrl_indices = [];

for f = 1:n_frames
    frame_mask = (original_pts(:,3) >= z_edges(f)) & (original_pts(:,3) < z_edges(f+1));
    frame_indices = find(frame_mask);
    
    % 从每帧中均匀选择控制点
    step = floor(length(frame_indices) / (n_ctrl_per_frame + 1));
    selected = frame_indices(step:step:step*n_ctrl_per_frame);
    ctrl_indices = [ctrl_indices; selected(:)];
end

src_ctrl = distorted_pts(ctrl_indices, :);
dst_ctrl = original_pts(ctrl_indices, :);
fprintf('  总共选择 %d 个控制点（每帧 %d 个）\n', length(ctrl_indices), n_ctrl_per_frame);

%% 4. 全局仿射变换校正
fprintf('\n>>> 步骤 4: 全局仿射变换校正\n');

options_global = struct();
options_global.validate = false;
options_global.visualize = false;

[corrected_global, M_global, stats_global] = affine12_main(distorted_pts, src_ctrl, dst_ctrl, options_global);

% 计算全局校正的RMSE
errors_global = corrected_global - original_pts;
rmse_global = sqrt(mean(sum(errors_global.^2, 2)));
fprintf('  全局仿射校正 RMSE: %.6f\n', rmse_global);

%% 5. 分块局部仿射变换校正
fprintf('\n>>> 步骤 5: 分块局部仿射变换校正\n');

options_local = struct();
options_local.n_blocks = n_frames;  % 与畸变帧数相同
options_local.block_axis = 'z';
options_local.overlap = 0.3;        % 增加重叠
options_local.blend_mode = 'hard';  % 使用硬分块，不混合
options_local.min_ctrl = 4;

[corrected_local, stats_local] = affine12_local(distorted_pts, src_ctrl, dst_ctrl, options_local);

% 计算分块校正的RMSE
errors_local = corrected_local - original_pts;
rmse_local = sqrt(mean(sum(errors_local.^2, 2)));
fprintf('  分块仿射校正 RMSE: %.6f\n', rmse_local);

%% 6. 可视化对比
fprintf('\n>>> 步骤 6: 可视化对比\n');

view_angle = [45, 25];

% 图1: 四图对比
figure('Name', '全局仿射 vs 分块仿射 对比', 'Position', [50, 100, 1600, 400]);

subplot(1, 4, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, original_pts(:,3), 'filled');
title('原始点云', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view(view_angle);

subplot(1, 4, 2);
scatter3(distorted_pts(:,1), distorted_pts(:,2), distorted_pts(:,3), 1, 'r', 'filled');
title('多帧拼接畸变', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view(view_angle);

subplot(1, 4, 3);
scatter3(corrected_global(:,1), corrected_global(:,2), corrected_global(:,3), 1, 'g', 'filled');
title(sprintf('全局仿射校正\nRMSE=%.4f', rmse_global), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view(view_angle);

subplot(1, 4, 4);
scatter3(corrected_local(:,1), corrected_local(:,2), corrected_local(:,3), 1, 'b', 'filled');
title(sprintf('分块仿射校正\nRMSE=%.4f', rmse_local), 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view(view_angle);

% 图2: 误差分布对比
figure('Name', '误差分布对比', 'Position', [100, 100, 1200, 400]);

subplot(1, 3, 1);
error_mag_global = sqrt(sum(errors_global.^2, 2));
error_mag_local = sqrt(sum(errors_local.^2, 2));
histogram(error_mag_global, 50, 'FaceColor', 'g', 'FaceAlpha', 0.7);
hold on;
histogram(error_mag_local, 50, 'FaceColor', 'b', 'FaceAlpha', 0.7);
hold off;
title('误差分布直方图', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('误差大小'); ylabel('频数');
legend(sprintf('全局 (RMSE=%.4f)', rmse_global), ...
       sprintf('分块 (RMSE=%.4f)', rmse_local), 'Location', 'best');
grid on;

subplot(1, 3, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, error_mag_global, 'filled');
title('全局仿射误差空间分布', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
colorbar; axis equal; grid on; view(view_angle);

subplot(1, 3, 3);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, error_mag_local, 'filled');
title('分块仿射误差空间分布', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
colorbar; axis equal; grid on; view(view_angle);

%% 7. 输出统计对比
fprintf('\n============================================================\n');
fprintf('  校正效果对比总结\n');
fprintf('============================================================\n');
fprintf('                    全局仿射      分块仿射\n');
fprintf('  RMSE:            %.6f      %.6f\n', rmse_global, rmse_local);
fprintf('  最大误差:        %.6f      %.6f\n', max(error_mag_global), max(error_mag_local));
fprintf('  平均误差:        %.6f      %.6f\n', mean(error_mag_global), mean(error_mag_local));
fprintf('  改进倍数:        -             %.1fx\n', rmse_global / rmse_local);
fprintf('============================================================\n');
