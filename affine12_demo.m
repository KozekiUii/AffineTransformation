%% AFFINE12_DEMO 12参数仿射变换点云畸变消除演示脚本
%
% 本脚本演示如何使用12参数仿射变换算法进行点云畸变消除
% 包含:
%   1. 生成模拟点云数据
%   2. 应用已知畸变
%   3. 使用算法恢复原始点云
%   4. 验证恢复精度
%
% 作者: 自动生成
% 日期: 2026-01-19

clear; clc; close all;

fprintf('============================================================\n');
fprintf('   12参数仿射变换点云畸变消除 - 演示脚本\n');
fprintf('============================================================\n\n');

%% 1. 生成模拟点云数据
fprintf('>>> 步骤 1: 生成模拟点云数据\n');

% 生成一个简单的建筑物形状点云
rng(42);  % 固定随机种子以确保可重复性

% 地面点
ground_pts = [rand(500, 1)*50-25, rand(500, 1)*50-25, zeros(500, 1)];

% 建筑物顶面
building_x = rand(200, 1)*10 - 5;
building_y = rand(200, 1)*10 - 5;
building_z = ones(200, 1) * 15;
building_top = [building_x, building_y, building_z];

% 建筑物侧面
wall1 = [-5*ones(100,1), rand(100,1)*10-5, rand(100,1)*15];
wall2 = [5*ones(100,1), rand(100,1)*10-5, rand(100,1)*15];
wall3 = [rand(100,1)*10-5, -5*ones(100,1), rand(100,1)*15];
wall4 = [rand(100,1)*10-5, 5*ones(100,1), rand(100,1)*15];

% 合并点云
original_pts = [ground_pts; building_top; wall1; wall2; wall3; wall4];
N = size(original_pts, 1);
fprintf('  生成 %d 个点\n', N);

%% 2. 定义已知的畸变参数
fprintf('\n>>> 步骤 2: 定义畸变参数\n');

% 定义12参数（模拟传感器系统误差）
true_rotation = [deg2rad(0.5), deg2rad(-0.3), deg2rad(0.8)];  % 微小旋转
true_scale = [1.002, 0.998, 1.001];                            % 微小缩放偏差
true_shear = [deg2rad(0.2), deg2rad(-0.1), deg2rad(0.15)];     % 微小切变
true_translation = [0.5, -0.3, 0.2];                           % 平移

fprintf('  真实旋转角 (度): [%.4f, %.4f, %.4f]\n', rad2deg(true_rotation));
fprintf('  真实缩放因子: [%.6f, %.6f, %.6f]\n', true_scale);
fprintf('  真实切变角 (度): [%.4f, %.4f, %.4f]\n', rad2deg(true_shear));
fprintf('  真实平移: [%.4f, %.4f, %.4f]\n', true_translation);

% 构建畸变变换矩阵
M_distort = affine12_build_matrix(true_rotation, true_scale, true_shear, true_translation);
fprintf('\n  畸变变换矩阵:\n');
disp(M_distort);

%% 3. 应用畸变并添加噪声
fprintf('>>> 步骤 3: 应用畸变\n');

distorted_pts = affine12_apply(original_pts, M_distort);

% 添加测量噪声
noise_level = 0.01;  % 1cm噪声
noise = noise_level * randn(N, 3);
distorted_pts_noisy = distorted_pts + noise;

fprintf('  畸变已应用\n');
fprintf('  添加噪声水平: %.4f\n', noise_level);

%% 4. 选择控制点
fprintf('\n>>> 步骤 4: 选择控制点\n');

% 从点云中选择一些特征点作为控制点
% 这里选择建筑物的角点等特征位置
ctrl_indices = [1, 100, 200, 500, 600, 700, 800, 900];  % 选择8个点
n_ctrl = length(ctrl_indices);

src_ctrl = distorted_pts_noisy(ctrl_indices, :);  % 畸变点云中的控制点
dst_ctrl = original_pts(ctrl_indices, :);          % 真实位置（目标）

fprintf('  选择 %d 个控制点\n', n_ctrl);

%% 5. 执行畸变消除
fprintf('\n>>> 步骤 5: 执行畸变消除\n');

options = struct();
options.validate = true;
options.visualize = true;
options.thresholds = struct();
options.thresholds.scale_tolerance = 0.01;        % 1%
options.thresholds.shear_tolerance = deg2rad(1);  % 1度

[corrected_pts, M_estimated, stats] = affine12_main(distorted_pts_noisy, src_ctrl, dst_ctrl, options);

%% 6. 验证恢复精度
fprintf('\n>>> 步骤 6: 验证恢复精度\n');

% 计算与原始点云的误差
errors = corrected_pts - original_pts;
rmse_recovery = sqrt(mean(sum(errors.^2, 2)));
max_error = max(sqrt(sum(errors.^2, 2)));
mean_error = mean(sqrt(sum(errors.^2, 2)));

fprintf('\n恢复精度评估:\n');
fprintf('  RMSE: %.6f\n', rmse_recovery);
fprintf('  平均误差: %.6f\n', mean_error);
fprintf('  最大误差: %.6f\n', max_error);

% 比较估计参数与真实参数
fprintf('\n参数恢复对比:\n');
fprintf('  旋转角误差 (度): [%.6f, %.6f, %.6f]\n', ...
    rad2deg(stats.params.rotation_angles - true_rotation));
fprintf('  缩放因子误差: [%.6f, %.6f, %.6f]\n', ...
    stats.params.scale_factors - true_scale);
fprintf('  切变角误差 (度): [%.6f, %.6f, %.6f]\n', ...
    rad2deg(stats.params.shear_angles - true_shear));
fprintf('  平移误差: [%.6f, %.6f, %.6f]\n', ...
    stats.params.translation - true_translation);

%% 7. 生成对比可视化
fprintf('\n>>> 步骤 7: 生成详细对比可视化\n');

figure('Name', '畸变消除效果对比', 'Position', [50, 50, 1600, 600]);

% 原始点云
subplot(1, 4, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, original_pts(:,3), '.');
title('原始点云（真值）');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; colorbar;
view(30, 30);

% 畸变点云
subplot(1, 4, 2);
scatter3(distorted_pts_noisy(:,1), distorted_pts_noisy(:,2), distorted_pts_noisy(:,3), 1, distorted_pts_noisy(:,3), '.');
title('畸变点云（含噪声）');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; colorbar;
view(30, 30);

% 校正后点云
subplot(1, 4, 3);
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, corrected_pts(:,3), '.');
title('校正后点云');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; colorbar;
view(30, 30);

% 误差分布
subplot(1, 4, 4);
error_magnitudes = sqrt(sum(errors.^2, 2));
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, error_magnitudes, '.');
title('误差分布');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; 
c = colorbar;
c.Label.String = '误差大小';
view(30, 30);

%% 8. 误差直方图
figure('Name', '误差分析', 'Position', [100, 100, 800, 300]);

subplot(1, 2, 1);
histogram(error_magnitudes, 50);
xlabel('误差大小');
ylabel('点数');
title('误差分布直方图');
grid on;

subplot(1, 2, 2);
histogram(errors(:,1), 30, 'FaceAlpha', 0.5, 'DisplayName', 'X误差');
hold on;
histogram(errors(:,2), 30, 'FaceAlpha', 0.5, 'DisplayName', 'Y误差');
histogram(errors(:,3), 30, 'FaceAlpha', 0.5, 'DisplayName', 'Z误差');
hold off;
xlabel('误差');
ylabel('点数');
title('XYZ分量误差分布');
legend('Location', 'best');
grid on;

%% 总结
fprintf('\n============================================================\n');
fprintf('   演示完成\n');
fprintf('============================================================\n');
fprintf('关键结果:\n');
fprintf('  - 控制点RMSE: %.6f\n', stats.rmse);
fprintf('  - 全局恢复RMSE: %.6f\n', rmse_recovery);
fprintf('  - 参数验证: %s\n', conditional_str(stats.is_valid, '通过', '存在警告'));
fprintf('============================================================\n');

function str = conditional_str(condition, true_str, false_str)
    if condition
        str = true_str;
    else
        str = false_str;
    end
end
