function h = visualize_pointcloud(pts, varargin)
% VISUALIZE_POINTCLOUD 点云可视化工具函数
%
% 输入:
%   pts      - 点云 (N×3)
%   varargin - 可选参数对
%              'Color'      - 颜色 ('z', 'intensity', [R G B], 或 N×1 向量)
%              'MarkerSize' - 点大小 (默认1)
%              'Title'      - 图标题
%              'Colormap'   - 颜色映射 (默认'jet')
%              'ShowGrid'   - 是否显示网格 (默认true)
%              'AxisEqual'  - 是否等比例 (默认true)
%
% 输出:
%   h - 图形句柄
%
% 使用示例:
%   visualize_pointcloud(pts, 'Color', 'z', 'Title', '点云可视化');
%   visualize_pointcloud(pts, 'Color', intensity_values);
%
% 作者: 自动生成
% 日期: 2026-01-19

    % 解析输入参数
    p = inputParser;
    addRequired(p, 'pts');
    addParameter(p, 'Color', 'z');
    addParameter(p, 'MarkerSize', 1);
    addParameter(p, 'Title', '点云可视化');
    addParameter(p, 'Colormap', 'jet');
    addParameter(p, 'ShowGrid', true);
    addParameter(p, 'AxisEqual', true);
    addParameter(p, 'View', [30, 30]);
    addParameter(p, 'Figure', []);
    
    parse(p, pts, varargin{:});
    opts = p.Results;
    
    % 创建或使用现有图形
    if isempty(opts.Figure)
        h = figure('Name', opts.Title);
    else
        h = opts.Figure;
        figure(h);
    end
    
    % 确定颜色
    if ischar(opts.Color)
        switch lower(opts.Color)
            case 'z'
                color_values = pts(:, 3);
            case 'y'
                color_values = pts(:, 2);
            case 'x'
                color_values = pts(:, 1);
            otherwise
                color_values = pts(:, 3);
        end
    elseif isnumeric(opts.Color) && numel(opts.Color) == 3
        % RGB颜色
        scatter3(pts(:,1), pts(:,2), pts(:,3), opts.MarkerSize, opts.Color, '.');
        color_values = [];
    else
        % 自定义颜色值
        color_values = opts.Color;
    end
    
    % 绘制点云
    if ~isempty(color_values)
        scatter3(pts(:,1), pts(:,2), pts(:,3), opts.MarkerSize, color_values, '.');
        colormap(opts.Colormap);
        colorbar;
    end
    
    % 设置属性
    title(opts.Title);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    if opts.ShowGrid
        grid on;
    end
    
    if opts.AxisEqual
        axis equal;
    end
    
    view(opts.View);
    
    drawnow;
end
