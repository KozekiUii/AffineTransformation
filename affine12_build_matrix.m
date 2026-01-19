function M = affine12_build_matrix(rotation_angles, scale_factors, shear_angles, translation)
% AFFINE12_BUILD_MATRIX 根据12个物理参数构建4×4仿射变换矩阵
%
% 输入:
%   rotation_angles - 旋转角 [omega, phi, kappa] (弧度)，绕X、Y、Z轴
%   scale_factors   - 缩放因子 [lambda1, lambda2, lambda3]，X、Y、Z轴
%   shear_angles    - 切变角 [theta1, theta2, theta3] (弧度)，XY、XZ、YZ平面
%   translation     - 平移向量 [tx, ty, tz]
%
% 输出:
%   M - 4×4齐次变换矩阵
%
% 数学原理:
%   M = [R * S * H, T; 0 0 0 1]
%   其中:
%     R = Rz(kappa) * Ry(phi) * Rx(omega) 为旋转矩阵
%     S = diag(lambda1, lambda2, lambda3) 为缩放矩阵
%     H 为切变矩阵
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 解析参数
    omega = rotation_angles(1);  % 绕X轴旋转
    phi   = rotation_angles(2);  % 绕Y轴旋转
    kappa = rotation_angles(3);  % 绕Z轴旋转
    
    lambda1 = scale_factors(1);  % X轴缩放
    lambda2 = scale_factors(2);  % Y轴缩放
    lambda3 = scale_factors(3);  % Z轴缩放
    
    theta1 = shear_angles(1);    % XY平面切变
    theta2 = shear_angles(2);    % XZ平面切变
    theta3 = shear_angles(3);    % YZ平面切变
    
    tx = translation(1);
    ty = translation(2);
    tz = translation(3);
    
    % 构建旋转矩阵 R = Rz * Ry * Rx (ZYX欧拉角顺序)
    Rx = [1, 0, 0;
          0, cos(omega), -sin(omega);
          0, sin(omega), cos(omega)];
    
    Ry = [cos(phi), 0, sin(phi);
          0, 1, 0;
          -sin(phi), 0, cos(phi)];
    
    Rz = [cos(kappa), -sin(kappa), 0;
          sin(kappa), cos(kappa), 0;
          0, 0, 1];
    
    R = Rz * Ry * Rx;
    
    % 构建缩放矩阵 S
    S = diag([lambda1, lambda2, lambda3]);
    
    % 构建切变矩阵 H
    % 切变矩阵定义为:
    % H = [1,       tan(theta1), tan(theta2);
    %      0,       1,           tan(theta3);
    %      0,       0,           1          ]
    H = [1,           tan(theta1), tan(theta2);
         0,           1,           tan(theta3);
         0,           0,           1];
    
    % 组合仿射子矩阵 A = R * S * H
    A = R * S * H;
    
    % 构建4×4齐次变换矩阵
    M = eye(4);
    M(1:3, 1:3) = A;
    M(1:3, 4) = [tx; ty; tz];
end
