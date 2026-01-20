function [corrected_pts, stats] = affine12_local(distorted_pts, src_ctrl, dst_ctrl, options)
% AFFINE12_LOCAL 分块局部仿射变换点云畸变消除
%
% 与全局仿射变换不同，本函数将点云分割成多个区域，
% 每个区域独立估计仿射变换参数，从而更好地处理：
%   - 多帧拼接错误
%   - 局部非线性畸变
%   - 运动畸变
%
% 输入:
%   distorted_pts - 待校正的点云 (N×3)
%   src_ctrl      - 源控制点（在畸变点云中）(M×3)
%   dst_ctrl      - 目标控制点（真实位置）(M×3)
%   options       - 可选参数结构体
%       .n_blocks       - 分块数量 (默认: 4)
%       .block_axis     - 分块轴向 'x', 'y', 'z' (默认: 'z')
%       .overlap        - 重叠比例 (默认: 0.1)
%       .blend_mode     - 混合模式 'hard', 'linear', 'gaussian' (默认: 'linear')
%       .min_ctrl       - 每块最少控制点数 (默认: 4)
%
% 输出:
%   corrected_pts - 校正后的点云 (N×3)
%   stats         - 统计信息结构体
%       .block_rmse     - 每块的RMSE
%       .block_params   - 每块的变换参数
%       .global_rmse    - 全局RMSE（用于对比）
%
% 作者: 自动生成
% 日期: 2026-01-19

    fprintf('========================================\n');
    fprintf('  分块局部仿射变换点云畸变消除\n');
    fprintf('========================================\n\n');
    
    %% 默认参数
    if nargin < 4
        options = struct();
    end
    
    if ~isfield(options, 'n_blocks')
        options.n_blocks = 4;
    end
    
    if ~isfield(options, 'block_axis')
        options.block_axis = 'z';
    end
    
    if ~isfield(options, 'overlap')
        options.overlap = 0.1;  % 10% 重叠
    end
    
    if ~isfield(options, 'blend_mode')
        options.blend_mode = 'linear';
    end
    
    if ~isfield(options, 'min_ctrl')
        options.min_ctrl = 4;
    end
    
    n_blocks = options.n_blocks;
    N = size(distorted_pts, 1);
    
    %% 确定分块轴
    switch lower(options.block_axis)
        case 'x'
            axis_idx = 1;
        case 'y'
            axis_idx = 2;
        case 'z'
            axis_idx = 3;
        otherwise
            error('无效的分块轴向: %s', options.block_axis);
    end
    
    fprintf('分块配置:\n');
    fprintf('  分块数量: %d\n', n_blocks);
    fprintf('  分块轴向: %s\n', options.block_axis);
    fprintf('  重叠比例: %.1f%%\n', options.overlap * 100);
    fprintf('  混合模式: %s\n', options.blend_mode);
    
    %% 计算分块边界
    % 【关键修复】使用目标控制点的坐标来确定边界，而不是畸变点云
    % 因为畸变改变了坐标，使用畸变坐标会导致分块与原始帧不对齐
    dst_axis_vals = dst_ctrl(:, axis_idx);
    axis_min = min(dst_axis_vals);
    axis_max = max(dst_axis_vals);
    axis_range = axis_max - axis_min;
    
    block_size = axis_range / n_blocks;
    overlap_size = block_size * options.overlap;
    
    % 计算每个块的边界（带重叠）
    block_edges = zeros(n_blocks, 2);
    for b = 1:n_blocks
        block_start = axis_min + (b-1) * block_size - overlap_size;
        block_end = axis_min + b * block_size + overlap_size;
        block_edges(b, :) = [block_start, block_end];
    end
    
    fprintf('  分块边界 (基于目标控制点):\n');
    for b = 1:n_blocks
        fprintf('    块 %d: [%.4f, %.4f]\n', b, block_edges(b, 1), block_edges(b, 2));
    end
    
    %% 初始化输出
    corrected_pts = zeros(N, 3);
    weights = zeros(N, 1);  % 用于混合
    
    stats = struct();
    stats.block_rmse = zeros(n_blocks, 1);
    stats.block_params = cell(n_blocks, 1);
    stats.block_n_ctrl = zeros(n_blocks, 1);
    stats.block_n_pts = zeros(n_blocks, 1);
    
    %% 逐块处理
    fprintf('\n逐块处理:\n');
    
    for b = 1:n_blocks
        fprintf('  块 %d/%d: ', b, n_blocks);
        
        % 【关键】使用目标控制点坐标来确定哪些控制点属于这个块
        dst_ctrl_axis = dst_ctrl(:, axis_idx);
        ctrl_mask = (dst_ctrl_axis >= block_edges(b, 1)) & (dst_ctrl_axis <= block_edges(b, 2));
        n_block_ctrl = sum(ctrl_mask);
        
        % 根据控制点的源/目标对应关系，确定该块中的点云范围
        % 使用源控制点（畸变点云中的位置）来确定点云的分块
        if n_block_ctrl > 0
            block_src_ctrl = src_ctrl(ctrl_mask, :);
            src_axis_min = min(block_src_ctrl(:, axis_idx));
            src_axis_max = max(block_src_ctrl(:, axis_idx));
            % 扩展一点范围以包含更多点
            src_margin = (src_axis_max - src_axis_min) * 0.3;
            src_axis_min = src_axis_min - src_margin;
            src_axis_max = src_axis_max + src_margin;
        else
            % 如果该块没有控制点，使用整个点云范围
            src_axis_min = min(distorted_pts(:, axis_idx));
            src_axis_max = max(distorted_pts(:, axis_idx));
        end
        
        % 找出该块中的点（使用畸变点云坐标）
        axis_vals = distorted_pts(:, axis_idx);
        block_mask = (axis_vals >= src_axis_min) & (axis_vals <= src_axis_max);
        block_pts = distorted_pts(block_mask, :);
        n_block_pts = sum(block_mask);
        
        stats.block_n_pts(b) = n_block_pts;
        stats.block_n_ctrl(b) = n_block_ctrl;
        
        fprintf('%d 点, %d 控制点 -> ', n_block_pts, n_block_ctrl);
        
        % 检查控制点是否足够
        if n_block_ctrl < options.min_ctrl
            fprintf('控制点不足(%d<%d)，使用全局变换\n', n_block_ctrl, options.min_ctrl);
            % 使用所有控制点的全局变换
            [M_block, ~, rmse_block] = affine12_estimate(src_ctrl, dst_ctrl);
            stats.block_rmse(b) = rmse_block;
        else
            % 使用该块的局部控制点
            block_src_ctrl = src_ctrl(ctrl_mask, :);
            block_dst_ctrl = dst_ctrl(ctrl_mask, :);
            [M_block, ~, rmse_block] = affine12_estimate(block_src_ctrl, block_dst_ctrl);
            stats.block_rmse(b) = rmse_block;
            fprintf('RMSE=%.6f\n', rmse_block);
        end
        
        % 保存该块的变换矩阵
        stats.block_params{b} = M_block;
        
        % 应用变换到该块的所有点
        block_corrected = affine12_apply(block_pts, M_block);
        
        % 计算混合权重（使用该块的实际范围）
        block_center = (src_axis_min + src_axis_max) / 2;
        block_half_size = (src_axis_max - src_axis_min) / 2;
        
        switch lower(options.blend_mode)
            case 'hard'
                % 硬切换，无混合
                w = ones(n_block_pts, 1);
            case 'linear'
                % 线性混合，边缘权重逐渐降低
                dist_to_center = abs(block_pts(:, axis_idx) - block_center);
                w = max(0.1, 1 - dist_to_center / block_half_size);  % 最小权重0.1
            case 'gaussian'
                % 高斯混合
                dist_to_center = abs(block_pts(:, axis_idx) - block_center);
                sigma = block_half_size / 2;
                w = exp(-dist_to_center.^2 / (2 * sigma^2));
        end
        
        % 累加加权结果
        block_indices = find(block_mask);
        for i = 1:n_block_pts
            idx = block_indices(i);
            corrected_pts(idx, :) = corrected_pts(idx, :) + w(i) * block_corrected(i, :);
            weights(idx) = weights(idx) + w(i);
        end
    end
    
    %% 归一化（处理重叠区域）
    for i = 1:N
        if weights(i) > 0
            corrected_pts(i, :) = corrected_pts(i, :) / weights(i);
        else
            % 如果某点未被任何块覆盖（理论上不应发生）
            corrected_pts(i, :) = distorted_pts(i, :);
        end
    end
    
    %% 计算全局RMSE（用于对比）
    [~, ~, global_rmse] = affine12_estimate(src_ctrl, dst_ctrl);
    stats.global_rmse = global_rmse;
    
    fprintf('\n========================================\n');
    fprintf('  分块局部仿射变换完成\n');
    fprintf('========================================\n');
    fprintf('各块RMSE: %s\n', mat2str(stats.block_rmse', 4));
    fprintf('全局RMSE (对比): %.6f\n', global_rmse);
    fprintf('========================================\n');
end
