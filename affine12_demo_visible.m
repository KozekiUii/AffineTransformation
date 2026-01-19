%% AFFINE12_DEMO_VISIBLE 12参数仿射变换点云畸变消除演示脚本（可见畸变版）
%
% 本脚本使用更大的畸变参数，使得畸变效果在视觉上明显可见
%
% 作者: 自动生成
% 日期: 2026-01-19

clear; clc; close all;

fprintf('============================================================\n');
fprintf('   12参数仿射变换点云畸变消除 - 可见畸变演示\n');
fprintf('============================================================\n\n');

%% 1. 生成模拟点云数据
fprintf('>>> 步骤 1: 生成模拟点云数据\n');

rng(42);

% 地面点
ground_pts = [rand(500, 1)*50-25, rand(500, 1)*50-25, zeros(500, 1)];

% 建筑物
building_x = rand(200, 1)*10 - 5;
building_y = rand(200, 1)*10 - 5;
building_z = ones(200, 1) * 15;
building_top = [building_x, building_y, building_z];

wall1 = [-5*ones(100,1), rand(100,1)*10-5, rand(100,1)*15];
wall2 = [5*ones(100,1), rand(100,1)*10-5, rand(100,1)*15];
wall3 = [rand(100,1)*10-5, -5*ones(100,1), rand(100,1)*15];
wall4 = [rand(100,1)*10-5, 5*ones(100,1), rand(100,1)*15];

original_pts = [ground_pts; building_top; wall1; wall2; wall3; wall4];
N = size(original_pts, 1);
fprintf('  生成 %d 个点\n', N);

%% 2. 定义【明显可见】的畸变参数
fprintf('\n>>> 步骤 2: 定义【明显可见】的畸变参数\n');

% ========================================
% 关键改动：增大畸变参数！
% ========================================
true_rotation = [deg2rad(15), deg2rad(-10), deg2rad(20)];  % 大旋转角度
true_scale = [1.15, 0.85, 1.10];                            % 15%的缩放差异
true_shear = [deg2rad(10), deg2rad(-8), deg2rad(5)];        % 明显的切变
true_translation = [8, -6, 3];                              % 大平移

fprintf('  真实旋转角 (度): [%.1f, %.1f, %.1f]\n', rad2deg(true_rotation));
fprintf('  真实缩放因子: [%.2f, %.2f, %.2f]\n', true_scale);
fprintf('  真实切变角 (度): [%.1f, %.1f, %.1f]\n', rad2deg(true_shear));
fprintf('  真实平移: [%.1f, %.1f, %.1f]\n', true_translation);

% 构建畸变变换矩阵
M_distort = affine12_build_matrix(true_rotation, true_scale, true_shear, true_translation);

%% 3. 应用畸变
fprintf('\n>>> 步骤 3: 应用畸变\n');

distorted_pts = affine12_apply(original_pts, M_distort);

% 添加少量噪声
noise_level = 0.05;
noise = noise_level * randn(N, 3);
distorted_pts_noisy = distorted_pts + noise;

fprintf('  畸变已应用\n');

%% 4. 选择控制点
fprintf('\n>>> 步骤 4: 选择控制点\n');

ctrl_indices = [1, 100, 200, 500, 600, 700, 800, 900];
n_ctrl = length(ctrl_indices);

src_ctrl = distorted_pts_noisy(ctrl_indices, :);
dst_ctrl = original_pts(ctrl_indices, :);

fprintf('  选择 %d 个控制点\n', n_ctrl);

%% 5. 执行畸变消除（关闭默认可视化，稍后自定义）
fprintf('\n>>> 步骤 5: 执行畸变消除\n');

options = struct();
options.validate = true;
options.visualize = false;  % 关闭默认可视化
options.thresholds = struct();
options.thresholds.scale_tolerance = 0.20;   % 放宽到20%
options.thresholds.shear_tolerance = deg2rad(15);  % 放宽到15度

[corrected_pts, M_estimated, stats] = affine12_main(distorted_pts_noisy, src_ctrl, dst_ctrl, options);

%% 6. 【关键】可视化畸变效果对比
fprintf('\n>>> 步骤 6: 可视化畸变效果对比\n');

figure('Name', '畸变效果对比（明显可见）', 'Position', [50, 100, 1600, 500]);

% 使用相同的视角
view_angle = [45, 25];

% 子图1: 原始点云（真值）
subplot(1, 3, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, original_pts(:,3), 'filled');
title('原始点云（真值）', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
xlim([-40, 40]); ylim([-40, 40]); zlim([-5, 25]);
view(view_angle);
colorbar;

% 子图2: 畸变点云
subplot(1, 3, 2);
scatter3(distorted_pts_noisy(:,1), distorted_pts_noisy(:,2), distorted_pts_noisy(:,3), 3, 'r', 'filled');
title('畸变点云（旋转+缩放+切变+平移）', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
xlim([-40, 40]); ylim([-40, 40]); zlim([-5, 25]);
view(view_angle);

% 子图3: 校正后点云
subplot(1, 3, 3);
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 3, 'g', 'filled');
title('校正后点云', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
xlim([-40, 40]); ylim([-40, 40]); zlim([-5, 25]);
view(view_angle);

%% 7. 叠加对比图
figure('Name', '点云叠加对比', 'Position', [100, 100, 1200, 500]);

% 子图1: 原始 vs 畸变
subplot(1, 2, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 2, 'b', 'filled');
hold on;
scatter3(distorted_pts_noisy(:,1), distorted_pts_noisy(:,2), distorted_pts_noisy(:,3), 2, 'r', 'filled');
hold off;
title('蓝色=原始 | 红色=畸变', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
legend('原始点云', '畸变点云', 'Location', 'best');
view(view_angle);

% 子图2: 原始 vs 校正后
subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 2, 'b', 'filled');
hold on;
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 2, 'g', 'filled');
hold off;
title('蓝色=原始 | 绿色=校正后', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
legend('原始点云', '校正后点云', 'Location', 'best');
view(view_angle);

%% 8. 建筑物局部放大 + 叠加对比
fprintf('\n>>> 步骤 7: 建筑物局部放大 + 叠加对比\n');

figure('Name', '建筑物对比（关键：看叠加效果）', 'Position', [50, 50, 1600, 800]);

% 筛选建筑物区域的点
building_mask_orig = (original_pts(:,3) > 5);
building_mask_dist = (distorted_pts_noisy(:,3) > 5);
building_mask_corr = (corrected_pts(:,3) > 5);

% 第一行：分别显示
subplot(2, 3, 1);
scatter3(original_pts(building_mask_orig,1), original_pts(building_mask_orig,2), ...
    original_pts(building_mask_orig,3), 8, 'b', 'filled');
title('原始建筑物（蓝色）', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
xlim([-10, 10]); ylim([-10, 10]); zlim([0, 20]);

subplot(2, 3, 2);
scatter3(distorted_pts_noisy(building_mask_dist,1), distorted_pts_noisy(building_mask_dist,2), ...
    distorted_pts_noisy(building_mask_dist,3), 8, 'r', 'filled');
title('畸变后建筑物（红色）- 位置/形状都变了', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
xlim([-10, 40]); ylim([-20, 20]); zlim([0, 25]);

subplot(2, 3, 3);
scatter3(corrected_pts(building_mask_corr,1), corrected_pts(building_mask_corr,2), ...
    corrected_pts(building_mask_corr,3), 8, 'g', 'filled');
title('校正后建筑物（绿色）', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
xlim([-10, 10]); ylim([-10, 10]); zlim([0, 20]);

% ================================================================
% 第二行：叠加对比（关键！）
% ================================================================

% 子图4: 原始 vs 畸变 叠加
subplot(2, 3, 4);
scatter3(original_pts(building_mask_orig,1), original_pts(building_mask_orig,2), ...
    original_pts(building_mask_orig,3), 10, 'b', 'filled');
hold on;
scatter3(distorted_pts_noisy(building_mask_dist,1), distorted_pts_noisy(building_mask_dist,2), ...
    distorted_pts_noisy(building_mask_dist,3), 10, 'r', 'filled');
hold off;
title({'【蓝色+红色叠加】', '两个建筑物完全分离！'}, 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
legend('原始', '畸变', 'Location', 'best');

% 子图5: 原始 vs 校正后 叠加（应该高度重合！）
subplot(2, 3, 5);
scatter3(original_pts(building_mask_orig,1), original_pts(building_mask_orig,2), ...
    original_pts(building_mask_orig,3), 10, 'b', 'filled');
hold on;
scatter3(corrected_pts(building_mask_corr,1), corrected_pts(building_mask_corr,2), ...
    corrected_pts(building_mask_corr,3), 10, 'g', 'filled');
hold off;
title({'【蓝色+绿色叠加】', '两个建筑物几乎完全重合！'}, 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0, 0.6, 0]);
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
xlim([-10, 10]); ylim([-10, 10]); zlim([0, 20]);
legend('原始', '校正后', 'Location', 'best');

% 子图6: 用连线显示对应点的误差
subplot(2, 3, 6);
% 随机选择一些点显示误差连线
sample_idx = randperm(sum(building_mask_corr), min(50, sum(building_mask_corr)));
corr_building = corrected_pts(building_mask_corr, :);
orig_building = original_pts(building_mask_orig, :);

scatter3(orig_building(:,1), orig_building(:,2), orig_building(:,3), 5, 'b', 'filled');
hold on;
scatter3(corr_building(:,1), corr_building(:,2), corr_building(:,3), 5, 'g', 'filled');

% 绘制误差连线
for i = 1:length(sample_idx)
    idx = sample_idx(i);
    plot3([orig_building(idx,1), corr_building(idx,1)], ...
        [orig_building(idx,2), corr_building(idx,2)], ...
        [orig_building(idx,3), corr_building(idx,3)], 'r-', 'LineWidth', 1);
end
hold off;

title({'误差连线图', '红线越短=校正越准确'}, 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; view([30, 20]);
xlim([-10, 10]); ylim([-10, 10]); zlim([0, 20]);

%% 9. 误差统计
fprintf('\n>>> 步骤 8: 误差统计\n');

errors = corrected_pts - original_pts;
rmse_recovery = sqrt(mean(sum(errors.^2, 2)));

% 畸变程度统计
distortion_errors = distorted_pts - original_pts;
distortion_magnitude = sqrt(mean(sum(distortion_errors.^2, 2)));

fprintf('\n========================================\n');
fprintf('  对比统计\n');
fprintf('========================================\n');
fprintf('畸变前后差异 (RMSE): %.4f\n', distortion_magnitude);
fprintf('校正后误差 (RMSE):  %.4f\n', rmse_recovery);
fprintf('误差减少倍数: %.1fx\n', distortion_magnitude / rmse_recovery);
fprintf('========================================\n');

%% 10. 参数恢复对比
fprintf('\n>>> 参数恢复精度:\n');
fprintf('  旋转角恢复误差 (度): [%.4f, %.4f, %.4f]\n', ...
    rad2deg(stats.params.rotation_angles - true_rotation));
fprintf('  缩放因子恢复误差: [%.6f, %.6f, %.6f]\n', ...
    stats.params.scale_factors - true_scale);

fprintf('\n============================================================\n');
fprintf('   演示完成 - 现在您应该能清晰看到畸变效果！\n');
fprintf('============================================================\n');
