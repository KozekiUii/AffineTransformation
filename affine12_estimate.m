function [M, residuals, rmse] = affine12_estimate(src_pts, dst_pts)
% AFFINE12_ESTIMATE 使用最小二乘法估计12参数仿射变换矩阵
%
% 输入:
%   src_pts - 源点云控制点 (N×3)，N >= 4
%   dst_pts - 目标点云控制点 (N×3)，N >= 4
%
% 输出:
%   M         - 4×4齐次变换矩阵
%   residuals - 残差向量 (N×3)
%   rmse      - 均方根误差
%
% 数学原理:
%   12参数仿射变换: [x'; y'; z'] = A * [x; y; z] + T
%   其中 A 为 3×3 仿射矩阵（9参数），T 为平移向量（3参数）
%
%   线性化为: dst = src * A' + ones(N,1) * T'
%   即: [x' y' z'] = [x y z 1] * [a11 a21 a31; a12 a22 a32; a13 a23 a33; tx ty tz]
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 验证输入
    if size(src_pts, 1) ~= size(dst_pts, 1)
        error('源点和目标点数量必须相同');
    end
    
    N = size(src_pts, 1);
    if N < 4
        error('至少需要4对控制点进行12参数仿射变换估计');
    end
    
    % 检查共面性
    if check_coplanarity(src_pts)
        warning('源控制点接近共面，可能导致求解不稳定');
    end
    
    % 构建设计矩阵
    % 对于每个点: [x' y' z'] = [x y z 1] * P
    % 其中 P 是 4×3 的参数矩阵
    
    % 设计矩阵 A_design: N×4
    A_design = [src_pts, ones(N, 1)];
    
    % 使用最小二乘求解每个目标坐标
    % dst_pts(:,1) = A_design * P(:,1)
    % dst_pts(:,2) = A_design * P(:,2)
    % dst_pts(:,3) = A_design * P(:,3)
    
    % 求解参数矩阵 P (4×3)
    P = A_design \ dst_pts;
    
    % 构建4×4齐次变换矩阵
    % M = [A  T]
    %     [0  1]
    % 其中 A = P(1:3,:)' 为3×3仿射矩阵
    %      T = P(4,:)'   为3×1平移向量
    
    A = P(1:3, :)';  % 3×3 仿射子矩阵
    T = P(4, :)';    % 3×1 平移向量
    
    M = eye(4);
    M(1:3, 1:3) = A;
    M(1:3, 4) = T;
    
    % 计算残差
    dst_estimated = (A * src_pts' + T)';
    residuals = dst_pts - dst_estimated;
    
    % 计算RMSE
    rmse = sqrt(mean(sum(residuals.^2, 2)));
    
    % 输出诊断信息
    fprintf('仿射变换参数估计完成:\n');
    fprintf('  控制点数量: %d\n', N);
    fprintf('  RMSE: %.6f\n', rmse);
    fprintf('  最大残差: %.6f\n', max(sqrt(sum(residuals.^2, 2))));
end

function is_coplanar = check_coplanarity(pts)
% CHECK_COPLANARITY 检查点集是否共面
%
% 使用SVD分析点集的主成分，如果第三个奇异值接近于零，则点集共面

    % 中心化
    centroid = mean(pts, 1);
    pts_centered = pts - centroid;
    
    % SVD分解
    [~, S, ~] = svd(pts_centered, 'econ');
    singular_values = diag(S);
    
    % 如果第三个奇异值相对于第一个奇异值很小，则认为共面
    if length(singular_values) >= 3
        ratio = singular_values(3) / singular_values(1);
        is_coplanar = ratio < 1e-6;
    else
        is_coplanar = true;
    end
end
