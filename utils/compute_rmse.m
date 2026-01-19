function [rmse, errors] = compute_rmse(pts1, pts2)
% COMPUTE_RMSE 计算两个点集之间的均方根误差
%
% 输入:
%   pts1 - 点集1 (N×3)
%   pts2 - 点集2 (N×3)
%
% 输出:
%   rmse   - 均方根误差
%   errors - 每个点的误差向量 (N×3)
%
% 作者: 自动生成
% 日期: 2026-01-19

    if size(pts1, 1) ~= size(pts2, 1)
        error('两个点集的点数必须相同');
    end
    
    if size(pts1, 2) ~= 3 || size(pts2, 2) ~= 3
        error('点集必须为N×3矩阵');
    end
    
    % 计算误差向量
    errors = pts1 - pts2;
    
    % 计算每个点的欧氏距离
    distances = sqrt(sum(errors.^2, 2));
    
    % 计算RMSE
    rmse = sqrt(mean(distances.^2));
end
