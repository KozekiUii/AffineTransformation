%% CAR_DEMO 使用真实汽车点云验证12参数仿射变换畸变消除算法
%
% 本脚本加载 car 文件夹中的真实点云数据，模拟畸变，然后验证算法的校正效果
%
% 作者: 自动生成
% 日期: 2026-01-19

clear; clc; close all;

fprintf('============================================================\n');
fprintf('   12参数仿射变换点云畸变消除 - 飞机点云验证\n');
fprintf('============================================================\n\n');

%% 1. 加载真实点云数据
fprintf('>>> 步骤 1: 加载飞机点云数据\n');

% 读取第一个汽车点云文件
data_file = fullfile(pwd, 'airplane', 'airplane_0010.txt');
if ~exist(data_file, 'file')
    error('找不到点云文件: %s\n请确保在正确的目录下运行。', data_file);
end

% 加载数据 (格式: x, y, z, nx, ny, nz)
raw_data = load(data_file);
original_pts = raw_data(:, 1:3);  % 只取坐标部分
normals = raw_data(:, 4:6);       % 法向量（可选使用）

N = size(original_pts, 1);
fprintf('  加载点云: %s\n', data_file);
fprintf('  点数: %d\n', N);
fprintf('  坐标范围:\n');
fprintf('    X: [%.4f, %.4f]\n', min(original_pts(:,1)), max(original_pts(:,1)));
fprintf('    Y: [%.4f, %.4f]\n', min(original_pts(:,2)), max(original_pts(:,2)));
fprintf('    Z: [%.4f, %.4f]\n', min(original_pts(:,3)), max(original_pts(:,3)));

%% 2. 定义模拟畸变参数
fprintf('\n>>> 步骤 2: 定义模拟畸变参数\n');

% ========================================================================
% 畸变类型选择（可组合使用）
% ========================================================================
distortion_type = 'multiframe';  % 'affine', 'motion', 'nonlinear', 'multiframe'

% ========================================================================
% 类型1: 全局仿射畸变（线性变换，不会撕裂）
% ========================================================================
if strcmp(distortion_type, 'affine')
    fprintf('  【全局仿射畸变】- 整体旋转/缩放/切变\n');
    true_rotation = [deg2rad(25), deg2rad(-18), deg2rad(30)];
    true_scale = [1.35, 0.70, 1.20];
    true_shear = [deg2rad(15), deg2rad(-12), deg2rad(10)];
    true_translation = [0.3, -0.25, 0.15];
    
    M_distort = affine12_build_matrix(true_rotation, true_scale, true_shear, true_translation);
    distorted_pts = affine12_apply(original_pts, M_distort);
end

% ========================================================================
% 类型2: 运动畸变（Motion Distortion）- 会导致拉伸/撕裂
% 模拟: LiDAR旋转扫描时，物体或传感器在移动
% ========================================================================
if strcmp(distortion_type, 'motion')
    fprintf('  【运动畸变】- 模拟扫描过程中的运动\n');
    
    % 模拟扫描顺序（假设按Y坐标顺序扫描）
    [~, scan_order] = sort(original_pts(:, 2));  % 按Y排序模拟扫描顺序
    t = (1:N)' / N;  % 归一化时间 [0, 1]
    
    % 运动参数（随时间变化的变换）
    motion_velocity = [0.5, 0.3, 0.1];      % 平移速度 (单位/扫描周期)
    motion_rotation_rate = deg2rad(20);     % 旋转速率 (弧度/扫描周期)
    
    distorted_pts = original_pts;
    for i = 1:N
        idx = scan_order(i);
        ti = t(i);  % 该点的采集时刻
        
        % 时变平移
        dx = motion_velocity(1) * ti;
        dy = motion_velocity(2) * ti;
        dz = motion_velocity(3) * ti;
        
        % 时变旋转（绕Z轴）
        theta = motion_rotation_rate * ti;
        R = [cos(theta), -sin(theta), 0;
             sin(theta),  cos(theta), 0;
             0,           0,          1];
        
        % 应用变换
        pt = original_pts(idx, :)';
        pt_rotated = R * pt;
        distorted_pts(idx, :) = pt_rotated' + [dx, dy, dz];
    end
    
    % 用于后续参数估计的"近似"真值（取中间时刻的变换）
    true_rotation = [0, 0, motion_rotation_rate * 0.5];
    true_scale = [1, 1, 1];
    true_shear = [0, 0, 0];
    true_translation = motion_velocity * 0.5;
end

% ========================================================================
% 类型3: 非线性畸变（径向畸变）- 边缘变形更严重
% 模拟: 镜头畸变、距离测量误差
% ========================================================================
if strcmp(distortion_type, 'nonlinear')
    fprintf('  【非线性畸变】- 距离相关的径向变形\n');
    
    % 计算每个点到中心的距离
    center = mean(original_pts, 1);
    rel_pts = original_pts - center;
    r = sqrt(sum(rel_pts(:, 1:2).^2, 2));  % 径向距离（XY平面）
    r_max = max(r);
    r_norm = r / r_max;  % 归一化距离 [0, 1]
    
    % 非线性畸变系数（桶形/枕形畸变）
    k1 = 0.3;   % 径向畸变系数
    k2 = 0.1;   % 高阶径向畸变
    
    % 径向缩放因子
    scale_factor = 1 + k1 * r_norm.^2 + k2 * r_norm.^4;
    
    % 应用畸变
    distorted_pts = center + rel_pts .* scale_factor;
    
    % 非线性畸变没有简单的"真值"参数
    true_rotation = [0, 0, 0];
    true_scale = [1, 1, 1];
    true_shear = [0, 0, 0];
    true_translation = [0, 0, 0];
end

% ========================================================================
% 类型4: 多帧拼接错误 - 部分点云块位移
% 模拟: ICP配准失败导致的局部错位
% ========================================================================
if strcmp(distortion_type, 'multiframe')
    fprintf('  【多帧拼接错误】- 部分区域配准失败\n');

    distorted_pts = original_pts;

    % 将点云分成若干"帧"（按Z坐标分层）
    n_frames = 4;
    z_min = min(original_pts(:, 3));
    z_max = max(original_pts(:, 3));
    z_edges = linspace(z_min, z_max, n_frames + 1);

    % 每帧添加随机位移（模拟配准误差）
    rng(42);  % 固定随机种子
    for f = 1:n_frames
        frame_mask = (original_pts(:,3) >= z_edges(f)) & (original_pts(:,3) < z_edges(f+1));

        % 随机刚体变换
        frame_translation = 0.1 * (rand(1, 3) - 0.5);  % 随机平移
        frame_rotation = deg2rad(5) * (rand(1, 3) - 0.5);  % 随机旋转

        M_frame = affine12_build_matrix(frame_rotation, [1,1,1], [0,0,0], frame_translation);
        distorted_pts(frame_mask, :) = affine12_apply(original_pts(frame_mask, :), M_frame);
    end

    % 多帧错误没有简单的"真值"参数
    true_rotation = [0, 0, 0];
    true_scale = [1, 1, 1];
    true_shear = [0, 0, 0];
    true_translation = [0, 0, 0];
end

fprintf('  畸变类型: %s\n', distortion_type);

%% 3. 添加测量噪声
fprintf('\n>>> 步骤 3: 添加测量噪声\n');

% 添加少量测量噪声
noise_level = 0.001;  % 非常小的噪声，模拟测量精度
noise = noise_level * randn(N, 3);
distorted_pts_noisy = distorted_pts + noise;

fprintf('  添加噪声水平: %.4f\n', noise_level);

%% 4. 选择控制点
fprintf('\n>>> 步骤 4: 选择控制点\n');

% 使用均匀采样策略选择控制点
n_ctrl = 10;  % 控制点数量

% 按照点云索引均匀采样
ctrl_step = floor(N / n_ctrl);
ctrl_indices = 1:ctrl_step:N;
ctrl_indices = ctrl_indices(1:n_ctrl);

src_ctrl = distorted_pts_noisy(ctrl_indices, :);
dst_ctrl = original_pts(ctrl_indices, :);

fprintf('  选择 %d 个控制点\n', n_ctrl);
fprintf('  控制点采样间隔: %d\n', ctrl_step);

%% 5. 执行畸变消除
fprintf('\n>>> 步骤 5: 执行畸变消除\n');

options = struct();
options.validate = true;
options.visualize = false;  % 稍后自定义可视化
options.thresholds = struct();
options.thresholds.scale_tolerance = 0.50;    % 放宽到50%（因为畸变很大）
options.thresholds.shear_tolerance = deg2rad(20);  % 放宽到20度

[corrected_pts, M_estimated, stats] = affine12_main(distorted_pts_noisy, src_ctrl, dst_ctrl, options);

%% 6. 可视化结果
fprintf('\n>>> 步骤 6: 可视化结果\n');

% 图1: 点云三视图对比
figure('Name', '汽车点云畸变消除结果', 'Position', [50, 100, 1600, 500]);

view_angle = [45, 25];

subplot(1, 3, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, original_pts(:,3), 'filled');
title('原始点云（真值）', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on; colorbar;
view(view_angle);

subplot(1, 3, 2);
scatter3(distorted_pts_noisy(:,1), distorted_pts_noisy(:,2), distorted_pts_noisy(:,3), 1, 'r', 'filled');
title('畸变点云', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
view(view_angle);

subplot(1, 3, 3);
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, 'g', 'filled');
title('校正后点云', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
view(view_angle);

% 图2: 叠加对比
figure('Name', '点云叠加对比', 'Position', [100, 100, 1200, 500]);

subplot(1, 2, 1);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, 'b', 'filled');
hold on;
scatter3(distorted_pts_noisy(:,1), distorted_pts_noisy(:,2), distorted_pts_noisy(:,3), 1, 'r', 'filled');
hold off;
title({'蓝色=原始 | 红色=畸变', '两个点云应该分离'}, 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
legend('原始点云', '畸变点云', 'Location', 'best');
view(view_angle);

subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 1, 'b', 'filled');
hold on;
scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, 'g', 'filled');
hold off;
title({'蓝色=原始 | 绿色=校正后', '两个点云应该重合'}, 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
legend('原始点云', '校正后点云', 'Location', 'best');
view(view_angle);

%% 7. 误差统计
fprintf('\n>>> 步骤 7: 误差统计\n');

errors = corrected_pts - original_pts;
error_magnitudes = sqrt(sum(errors.^2, 2));
rmse_recovery = sqrt(mean(error_magnitudes.^2));

distortion_errors = distorted_pts - original_pts;
distortion_magnitudes = sqrt(sum(distortion_errors.^2, 2));
rmse_distortion = sqrt(mean(distortion_magnitudes.^2));

fprintf('\n========================================\n');
fprintf('  误差统计\n');
fprintf('========================================\n');
fprintf('畸变前后差异 (RMSE): %.6f\n', rmse_distortion);
fprintf('校正后残余误差 (RMSE): %.6f\n', rmse_recovery);
fprintf('误差减少倍数: %.1fx\n', rmse_distortion / rmse_recovery);
fprintf('最大残余误差: %.6f\n', max(error_magnitudes));
fprintf('平均残余误差: %.6f\n', mean(error_magnitudes));
fprintf('========================================\n');

% 图3: 误差分布
figure('Name', '误差分析', 'Position', [150, 100, 1000, 400]);

subplot(1, 2, 1);
histogram(error_magnitudes, 50, 'FaceColor', [0.2, 0.6, 0.9]);
title('校正后误差分布', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('误差大小'); ylabel('频数');
xline(rmse_recovery, 'r--', 'LineWidth', 2, 'Label', sprintf('RMSE=%.4f', rmse_recovery));
grid on;

subplot(1, 2, 2);
scatter3(original_pts(:,1), original_pts(:,2), original_pts(:,3), 3, error_magnitudes, 'filled');
title('误差空间分布', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
colorbar; caxis([0, max(error_magnitudes)]);
axis equal; grid on;
view(view_angle);

%% 8. 参数恢复精度
fprintf('\n>>> 步骤 8: 参数恢复精度\n');
fprintf('  真实旋转角 (度): [%.4f, %.4f, %.4f]\n', rad2deg(true_rotation));
fprintf('  估计旋转角 (度): [%.4f, %.4f, %.4f]\n', rad2deg(stats.params.rotation_angles));
fprintf('  旋转角误差 (度): [%.4f, %.4f, %.4f]\n', ...
    rad2deg(stats.params.rotation_angles - true_rotation));

fprintf('\n  真实缩放因子: [%.6f, %.6f, %.6f]\n', true_scale);
fprintf('  估计缩放因子: [%.6f, %.6f, %.6f]\n', stats.params.scale_factors);
fprintf('  缩放因子误差: [%.6f, %.6f, %.6f]\n', ...
    stats.params.scale_factors - true_scale);

fprintf('\n============================================================\n');
fprintf('   汽车点云验证完成！\n');
fprintf('============================================================\n');
