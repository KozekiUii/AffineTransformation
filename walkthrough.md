# 12参数仿射变换点云畸变消除 MATLAB 算法

## 概述

本算法实现了基于 12 参数仿射变换的点云畸变消除功能，用于解决 LiDAR 硬件系统中的残留系统偏差。

**适用场景**：
- ✅ 硬件系统误差（轴向非正交、刻度盘误差）
- ✅ 航带平差（扫描带重叠区失配）
- ✅ 传感器标定残余

**局限性**：
- ⚠️ 运动畸变（部分校正）
- ❌ 非线性畸变
- ❌ 多帧拼接错误

---

## 文件结构

```
e:\Study\AffineTransformation\
├── 核心函数
│   ├── affine12_main.m           # 主函数：完整畸变消除流程
│   ├── affine12_estimate.m       # 最小二乘估计 12 参数
│   ├── affine12_build_matrix.m   # 构建 4×4 仿射变换矩阵
│   ├── affine12_decompose.m      # 矩阵分解（提取 R、S、H、T）
│   ├── affine12_apply.m          # 应用变换至点云
│   └── affine12_validate.m       # 物理参数校核
│
├── 扩展功能
│   └── affine12_local.m          # 分块局部仿射变换（实验性）
│
├── 演示脚本
│   ├── affine12_demo.m           # 微小畸变演示（模拟真实硬件误差）
│   ├── affine12_demo_visible.m   # 明显畸变演示（便于可视化理解）
│   ├── car_demo.m                # 真实点云验证（多种畸变类型）
│   └── local_vs_global_demo.m    # 全局 vs 分块仿射对比
│
├── 数据
│   ├── car/                      # 汽车点云 (297 个文件)
│   └── airplane/                 # 飞机点云
│
├── 工具函数
│   └── utils/
│       ├── check_coplanarity.m
│       ├── compute_rmse.m
│       └── visualize_pointcloud.m
│
└── 文档
    ├── walkthrough.md                          # 本文档
    └── 仿射变换在点云图像畸变消除中的应用研究报告.md
```

---

## 快速开始

### 运行演示

```matlab
cd 'e:\Study\AffineTransformation'

% 1. 可视化效果明显的演示
affine12_demo_visible

% 2. 真实点云验证（支持多种畸变类型）
car_demo

% 3. 全局 vs 分块仿射对比
local_vs_global_demo
```

### 基本用法

```matlab
% 加载点云和控制点
distorted = load('distorted_cloud.txt');   % N×3 畸变点云
src_ctrl = load('src_control.txt');        % M×3 源控制点
dst_ctrl = load('dst_control.txt');        % M×3 目标控制点

% 执行畸变消除
[corrected, M, stats] = affine12_main(distorted, src_ctrl, dst_ctrl);

% 查看结果
fprintf('RMSE: %.4f\n', stats.rmse);
```

---

## 支持的畸变类型

`car_demo.m` 支持 4 种畸变模式（修改第 41 行）：

| 类型     | 代码           | 特点                | 仿射效果 |
| -------- | -------------- | ------------------- | -------- |
| 全局仿射 | `'affine'`     | 旋转+缩放+切变+平移 | ✅ 完美   |
| 运动畸变 | `'motion'`     | 扫描中物体移动      | ⚠️ 部分   |
| 非线性   | `'nonlinear'`  | 径向变形            | ❌ 差     |
| 多帧拼接 | `'multiframe'` | 分层错位            | ❌ 差     |

---

## 核心函数 API

### affine12_main

```matlab
[corrected_pts, M, stats] = affine12_main(distorted_pts, src_ctrl, dst_ctrl, options)
```

### affine12_local（分块仿射）

```matlab
[corrected_pts, stats] = affine12_local(distorted_pts, src_ctrl, dst_ctrl, options)

% 选项
options.n_blocks = 4;           % 分块数量
options.block_axis = 'z';       % 分块轴向
options.blend_mode = 'hard';    % 'hard', 'linear', 'gaussian'
options.min_ctrl = 4;           % 每块最少控制点
```

---

## 实验验证结果

### 全局仿射畸变

| 指标          | 值     |
| ------------- | ------ |
| 控制点 RMSE   | < 0.01 |
| 全局恢复 RMSE | < 0.01 |
| 误差减少倍数  | > 100x |

### 多帧拼接错误

| 方法     | RMSE   | 改进 |
| -------- | ------ | ---- |
| 全局仿射 | 0.0597 | 基准 |
| 分块仿射 | 0.0553 | +7%  |

**结论**：仿射变换对多帧拼接效果有限，建议使用 ICP 配准。

---

## 常见问题

### Q: 为什么看不出畸变效果？

**A**: `affine12_demo.m` 模拟真实硬件误差（旋转<1°），肉眼难以区分。请运行 `affine12_demo_visible`。

### Q: 为什么分块仿射效果不好？

**A**: 分块仿射适用于全局仿射的分区域改进。对于多帧拼接等非全局畸变，12 参数模型本身不适合，建议使用：
- ICP 点云配准
- 6 参数刚体变换
- 薄板样条 (TPS)

### Q: 控制点从哪里来？

**A**: 真实场景中需要外部高精度参考：
- GPS/RTK 测量点
- 全站仪测量点
- 已知 CAD/BIM 模型特征点
