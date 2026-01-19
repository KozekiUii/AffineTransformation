function transformed_pts = affine12_apply(pts, M)
% AFFINE12_APPLY 将仿射变换应用到点云
%
% 输入:
%   pts - 输入点云 (N×3)
%   M   - 4×4齐次变换矩阵
%
% 输出:
%   transformed_pts - 变换后的点云 (N×3)
%
% 数学原理:
%   对于每个点 p = [x; y; z; 1]
%   变换后: p' = M * p
%   即: [x'; y'; z'; 1] = M * [x; y; z; 1]
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 验证输入
    if size(pts, 2) ~= 3
        error('输入点云必须为N×3矩阵');
    end
    
    if ~isequal(size(M), [4, 4])
        error('变换矩阵必须为4×4矩阵');
    end
    
    N = size(pts, 1);
    
    % 转换为齐次坐标 (N×4)
    pts_homo = [pts, ones(N, 1)];
    
    % 应用变换
    % transformed = (M * pts_homo')'
    transformed_homo = (M * pts_homo')';
    
    % 提取3D坐标
    transformed_pts = transformed_homo(:, 1:3);
    
    % 如果最后一列不是1，进行归一化（对于仿射变换通常不需要）
    w = transformed_homo(:, 4);
    if any(abs(w - 1) > 1e-10)
        transformed_pts = transformed_pts ./ w;
    end
end
