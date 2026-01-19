function [corrected_pts, M, stats] = affine12_main(distorted_pts, src_ctrl, dst_ctrl, options)
% AFFINE12_MAIN 点云畸变消除主函数
%
% 输入:
%   distorted_pts - 待校正的点云 (N×3)
%   src_ctrl      - 源控制点（在畸变点云中）(M×3)，M >= 4
%   dst_ctrl      - 目标控制点（真实位置）(M×3)，M >= 4
%   options       - 可选参数结构体
%                   .validate      是否进行参数验证（默认true）
%                   .visualize     是否可视化（默认true）
%                   .thresholds    验证阈值（参见affine12_validate）
%
% 输出:
%   corrected_pts - 校正后的点云 (N×3)
%   M             - 4×4仿射变换矩阵
%   stats         - 统计信息结构体
%                   .rmse          控制点RMSE
%                   .residuals     控制点残差
%                   .params        分解后的12参数
%                   .is_valid      参数是否合理
%
% 使用示例:
%   % 加载点云和控制点
%   distorted = load('distorted_cloud.txt');
%   src_ctrl = load('src_control_points.txt');
%   dst_ctrl = load('dst_control_points.txt');
%   
%   % 执行畸变消除
%   [corrected, M, stats] = affine12_main(distorted, src_ctrl, dst_ctrl);
%
% 作者: 自动生成
% 日期: 2026-01-19

    fprintf('========================================\n');
    fprintf('  12参数仿射变换点云畸变消除\n');
    fprintf('========================================\n\n');
    
    % 默认选项
    if nargin < 4
        options = struct();
    end
    
    if ~isfield(options, 'validate')
        options.validate = true;
    end
    
    if ~isfield(options, 'visualize')
        options.visualize = true;
    end
    
    % 输入验证
    fprintf('步骤 1/5: 输入数据验证...\n');
    
    if size(distorted_pts, 2) ~= 3
        error('点云数据必须为N×3矩阵');
    end
    
    if size(src_ctrl, 1) ~= size(dst_ctrl, 1)
        error('源控制点和目标控制点数量必须相同');
    end
    
    if size(src_ctrl, 1) < 4
        error('至少需要4对控制点');
    end
    
    fprintf('  点云大小: %d 点\n', size(distorted_pts, 1));
    fprintf('  控制点对数: %d\n', size(src_ctrl, 1));
    
    % 估计仿射变换参数
    fprintf('\n步骤 2/5: 估计仿射变换参数...\n');
    [M, residuals, rmse] = affine12_estimate(src_ctrl, dst_ctrl);
    
    stats = struct();
    stats.rmse = rmse;
    stats.residuals = residuals;
    stats.M = M;
    
    % 矩阵分解
    fprintf('\n步骤 3/5: 变换矩阵分解...\n');
    [R, S, H, T, params] = affine12_decompose(M);
    stats.params = params;
    stats.R = R;
    stats.S = S;
    stats.H = H;
    stats.T = T;
    
    % 参数验证
    if options.validate
        fprintf('\n步骤 4/5: 参数物理校核...\n');
        if isfield(options, 'thresholds')
            [is_valid, warnings, report] = affine12_validate(params, options.thresholds);
        else
            [is_valid, warnings, report] = affine12_validate(params);
        end
        stats.is_valid = is_valid;
        stats.warnings = warnings;
        stats.validation_report = report;
    else
        fprintf('\n步骤 4/5: 跳过参数验证\n');
        stats.is_valid = true;
        stats.warnings = {};
    end
    
    % 应用变换
    fprintf('\n步骤 5/5: 应用变换到点云...\n');
    corrected_pts = affine12_apply(distorted_pts, M);
    fprintf('  变换完成，输出 %d 点\n', size(corrected_pts, 1));
    
    % 可视化
    if options.visualize
        fprintf('\n生成可视化...\n');
        visualize_results(distorted_pts, corrected_pts, src_ctrl, dst_ctrl, M);
    end
    
    % 输出总结
    fprintf('\n========================================\n');
    fprintf('  畸变消除完成\n');
    fprintf('========================================\n');
    fprintf('控制点RMSE: %.6f\n', rmse);
    if stats.is_valid
        fprintf('参数验证: ✓ 通过\n');
    else
        fprintf('参数验证: ✗ 存在警告\n');
    end
    fprintf('========================================\n');
end

function visualize_results(distorted_pts, corrected_pts, src_ctrl, dst_ctrl, M)
% VISUALIZE_RESULTS 可视化畸变消除结果

    figure('Name', '12参数仿射变换点云畸变消除结果', 'Position', [100, 100, 1400, 500]);
    
    % 子图1: 原始畸变点云
    subplot(1, 3, 1);
    scatter3(distorted_pts(:,1), distorted_pts(:,2), distorted_pts(:,3), 1, 'b', '.');
    hold on;
    scatter3(src_ctrl(:,1), src_ctrl(:,2), src_ctrl(:,3), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');
    hold off;
    title('原始畸变点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    legend('点云', '源控制点', 'Location', 'best');
    
    % 子图2: 校正后点云
    subplot(1, 3, 2);
    scatter3(corrected_pts(:,1), corrected_pts(:,2), corrected_pts(:,3), 1, 'g', '.');
    hold on;
    scatter3(dst_ctrl(:,1), dst_ctrl(:,2), dst_ctrl(:,3), 100, 'm', 'filled', 'MarkerEdgeColor', 'k');
    hold off;
    title('校正后点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    legend('校正点云', '目标控制点', 'Location', 'best');
    
    % 子图3: 控制点配准效果
    subplot(1, 3, 3);
    transformed_ctrl = affine12_apply(src_ctrl, M);
    
    % 绘制目标控制点
    scatter3(dst_ctrl(:,1), dst_ctrl(:,2), dst_ctrl(:,3), 100, 'm', 'filled', 'MarkerEdgeColor', 'k');
    hold on;
    % 绘制变换后的源控制点
    scatter3(transformed_ctrl(:,1), transformed_ctrl(:,2), transformed_ctrl(:,3), 80, 'c', 'filled');
    
    % 绘制残差向量
    for i = 1:size(dst_ctrl, 1)
        plot3([transformed_ctrl(i,1), dst_ctrl(i,1)], ...
              [transformed_ctrl(i,2), dst_ctrl(i,2)], ...
              [transformed_ctrl(i,3), dst_ctrl(i,3)], 'r-', 'LineWidth', 2);
    end
    hold off;
    
    title('控制点配准效果');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal; grid on;
    legend('目标控制点', '变换后源控制点', '残差', 'Location', 'best');
    
    drawnow;
end
