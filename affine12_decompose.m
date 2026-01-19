function [R, S, H, T, params] = affine12_decompose(M)
% AFFINE12_DECOMPOSE 将4×4仿射变换矩阵分解为旋转、缩放、切变和平移分量
%
% 输入:
%   M - 4×4齐次变换矩阵
%
% 输出:
%   R      - 3×3旋转矩阵
%   S      - 3×3缩放矩阵 (对角矩阵)
%   H      - 3×3切变矩阵 (上三角矩阵)
%   T      - 3×1平移向量
%   params - 结构体，包含所有12个物理参数:
%            .rotation_angles [omega, phi, kappa] (弧度)
%            .scale_factors   [lambda1, lambda2, lambda3]
%            .shear_angles    [theta1, theta2, theta3] (弧度)
%            .translation     [tx, ty, tz]
%
% 数学原理:
%   使用极分解和QR分解提取各分量
%   M = [A, T; 0, 1], 其中 A = R * S * H
%   1. 提取平移向量 T = M(1:3, 4)
%   2. 对 A 进行极分解: A = R * P，其中R为正交矩阵，P为正定对称矩阵
%   3. 对 P 进行QR分解提取缩放和切变
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 提取仿射子矩阵和平移向量
    A = M(1:3, 1:3);
    T = M(1:3, 4);
    
    % 极分解: A = R * P
    % 其中 R 为旋转矩阵，P 为正定对称矩阵
    [U, Sigma, V] = svd(A);
    R = U * V';
    
    % 确保R是正确的旋转矩阵（行列式为+1）
    if det(R) < 0
        R = -R;
        Sigma = -Sigma;
    end
    
    % P = V * Sigma * V'
    P = V * Sigma * V';
    
    % 对P进行Cholesky分解或QR分解来提取缩放和切变
    % 使用RQ分解: P = S_diag * H_upper
    % 这里我们使用一种简化方法：假设切变较小
    
    % 从P中提取缩放因子（对角元素的近似）
    % 对于小切变，P ≈ S * H，其中S是对角矩阵
    
    % 使用QR分解的转置形式来分解P
    % P' = Q * R_upper => P = R_upper' * Q' = L_lower * Q'
    % 然后从L_lower提取缩放和切变
    
    [Q, R_upper] = qr(P');
    L_lower = R_upper';
    
    % 从下三角矩阵提取参数
    % L = S * H_lower，需要转换为上三角形式
    
    % 提取缩放因子（对角元素）
    scale_factors = abs(diag(L_lower))';
    lambda1 = scale_factors(1);
    lambda2 = scale_factors(2);
    lambda3 = scale_factors(3);
    
    % 构建缩放矩阵
    S = diag(scale_factors);
    
    % 计算切变矩阵 H = S^(-1) * P * Q
    if all(scale_factors > 1e-10)
        H = S \ (L_lower * Q');
    else
        warning('缩放因子接近零，切变矩阵计算可能不稳定');
        H = eye(3);
    end
    
    % 从切变矩阵提取切变角
    % H = [1, tan(theta1), tan(theta2);
    %      0, 1,           tan(theta3);
    %      0, 0,           1          ]
    theta1 = atan(H(1, 2));
    theta2 = atan(H(1, 3));
    theta3 = atan(H(2, 3));
    
    % 从旋转矩阵提取欧拉角 (ZYX顺序)
    % R = Rz(kappa) * Ry(phi) * Rx(omega)
    phi = asin(-R(3, 1));
    
    if abs(cos(phi)) > 1e-6
        omega = atan2(R(3, 2), R(3, 3));
        kappa = atan2(R(2, 1), R(1, 1));
    else
        % 万向节锁定情况
        omega = atan2(-R(1, 2), R(2, 2));
        kappa = 0;
    end
    
    % 构建参数结构体
    params = struct();
    params.rotation_angles = [omega, phi, kappa];
    params.scale_factors = [lambda1, lambda2, lambda3];
    params.shear_angles = [theta1, theta2, theta3];
    params.translation = T';
    
    % 输出诊断信息
    fprintf('\n仿射变换矩阵分解结果:\n');
    fprintf('  旋转角 (度): omega=%.4f, phi=%.4f, kappa=%.4f\n', ...
        rad2deg(omega), rad2deg(phi), rad2deg(kappa));
    fprintf('  缩放因子: lambda1=%.6f, lambda2=%.6f, lambda3=%.6f\n', ...
        lambda1, lambda2, lambda3);
    fprintf('  切变角 (度): theta1=%.4f, theta2=%.4f, theta3=%.4f\n', ...
        rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
    fprintf('  平移向量: tx=%.4f, ty=%.4f, tz=%.4f\n', T(1), T(2), T(3));
end
