function is_coplanar = check_coplanarity(pts, tolerance)
% CHECK_COPLANARITY 检查点集是否共面
%
% 输入:
%   pts       - 点集 (N×3)
%   tolerance - 可选，判断阈值（默认1e-6）
%
% 输出:
%   is_coplanar - 布尔值，点集是否共面
%
% 数学原理:
%   使用SVD分析点集的主成分
%   如果第三个奇异值相对于第一个奇异值很小，则点集共面
%
% 作者: 自动生成
% 日期: 2026-01-19

    if nargin < 2
        tolerance = 1e-6;
    end
    
    N = size(pts, 1);
    
    if N < 4
        % 少于4个点必然共面
        is_coplanar = true;
        return;
    end
    
    % 中心化
    centroid = mean(pts, 1);
    pts_centered = pts - centroid;
    
    % SVD分解
    [~, S, ~] = svd(pts_centered, 'econ');
    singular_values = diag(S);
    
    % 如果第三个奇异值相对于第一个奇异值很小，则认为共面
    if length(singular_values) >= 3
        ratio = singular_values(3) / singular_values(1);
        is_coplanar = ratio < tolerance;
    else
        is_coplanar = true;
    end
    
    if is_coplanar
        fprintf('警告: 点集接近共面 (奇异值比: %.2e)\n', ratio);
    end
end
