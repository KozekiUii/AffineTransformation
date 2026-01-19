function [is_valid, warnings, report] = affine12_validate(params, thresholds)
% AFFINE12_VALIDATE 验证仿射变换参数的物理合理性
%
% 输入:
%   params     - 参数结构体（来自affine12_decompose）
%                .rotation_angles [omega, phi, kappa] (弧度)
%                .scale_factors   [lambda1, lambda2, lambda3]
%                .shear_angles    [theta1, theta2, theta3] (弧度)
%                .translation     [tx, ty, tz]
%   thresholds - 可选，阈值结构体
%                .scale_tolerance  缩放偏差容许范围（默认0.01，即1%）
%                .shear_tolerance  切变角容许范围（默认1度）
%                .rotation_max     最大旋转角（默认10度）
%
% 输出:
%   is_valid - 布尔值，参数是否在合理范围内
%   warnings - 警告信息元胞数组
%   report   - 详细验证报告结构体
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 默认阈值
    if nargin < 2 || isempty(thresholds)
        thresholds = struct();
    end
    
    if ~isfield(thresholds, 'scale_tolerance')
        thresholds.scale_tolerance = 0.01;  % 1%
    end
    
    if ~isfield(thresholds, 'shear_tolerance')
        thresholds.shear_tolerance = deg2rad(1);  % 1度
    end
    
    if ~isfield(thresholds, 'rotation_max')
        thresholds.rotation_max = deg2rad(10);  % 10度
    end
    
    % 初始化
    warnings = {};
    is_valid = true;
    report = struct();
    
    % 验证缩放因子
    scale_factors = params.scale_factors;
    scale_deviations = abs(scale_factors - 1);
    
    report.scale_factors = scale_factors;
    report.scale_deviations = scale_deviations;
    report.scale_deviations_percent = scale_deviations * 100;
    
    for i = 1:3
        if scale_deviations(i) > thresholds.scale_tolerance
            warnings{end+1} = sprintf(...
                '缩放因子 lambda%d=%.6f 偏离1的幅度(%.2f%%)超过阈值(%.2f%%)', ...
                i, scale_factors(i), scale_deviations(i)*100, thresholds.scale_tolerance*100);
            is_valid = false;
        end
    end
    
    % 验证切变角
    shear_angles = params.shear_angles;
    
    report.shear_angles_deg = rad2deg(shear_angles);
    
    for i = 1:3
        if abs(shear_angles(i)) > thresholds.shear_tolerance
            warnings{end+1} = sprintf(...
                '切变角 theta%d=%.4f° 超过阈值%.4f°', ...
                i, rad2deg(shear_angles(i)), rad2deg(thresholds.shear_tolerance));
            is_valid = false;
        end
    end
    
    % 验证旋转角
    rotation_angles = params.rotation_angles;
    
    report.rotation_angles_deg = rad2deg(rotation_angles);
    
    angle_names = {'omega', 'phi', 'kappa'};
    for i = 1:3
        if abs(rotation_angles(i)) > thresholds.rotation_max
            warnings{end+1} = sprintf(...
                '旋转角 %s=%.4f° 超过阈值%.4f°', ...
                angle_names{i}, rad2deg(rotation_angles(i)), rad2deg(thresholds.rotation_max));
            % 旋转角超过阈值只是警告，不影响有效性
        end
    end
    
    % 检查变换矩阵的条件数（数值稳定性）
    if isfield(params, 'condition_number')
        report.condition_number = params.condition_number;
        if params.condition_number > 1e6
            warnings{end+1} = sprintf(...
                '变换矩阵条件数%.2e过大，数值不稳定', params.condition_number);
        end
    end
    
    % 输出验证报告
    fprintf('\n=== 仿射变换参数验证报告 ===\n');
    fprintf('缩放因子偏差:\n');
    fprintf('  lambda1: %.6f (偏差 %.4f%%)\n', scale_factors(1), scale_deviations(1)*100);
    fprintf('  lambda2: %.6f (偏差 %.4f%%)\n', scale_factors(2), scale_deviations(2)*100);
    fprintf('  lambda3: %.6f (偏差 %.4f%%)\n', scale_factors(3), scale_deviations(3)*100);
    fprintf('切变角:\n');
    fprintf('  theta1: %.4f°\n', rad2deg(shear_angles(1)));
    fprintf('  theta2: %.4f°\n', rad2deg(shear_angles(2)));
    fprintf('  theta3: %.4f°\n', rad2deg(shear_angles(3)));
    fprintf('旋转角:\n');
    fprintf('  omega: %.4f°\n', rad2deg(rotation_angles(1)));
    fprintf('  phi: %.4f°\n', rad2deg(rotation_angles(2)));
    fprintf('  kappa: %.4f°\n', rad2deg(rotation_angles(3)));
    
    if is_valid
        fprintf('\n✓ 所有参数在合理范围内\n');
    else
        fprintf('\n✗ 发现以下问题:\n');
        for i = 1:length(warnings)
            fprintf('  - %s\n', warnings{i});
        end
    end
    fprintf('================================\n');
end
