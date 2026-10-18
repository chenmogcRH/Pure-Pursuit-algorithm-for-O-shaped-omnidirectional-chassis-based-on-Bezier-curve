# 算法说明

本文解释 `src/bezier.cpp` 与 `src/pure_pursuit.cpp` 的计算过程。它不是经典差速车的“曲率直接转轮速”公式；这里先求二维平移方向，再独立求朝向控制量。

## 1. 路径生成

对控制点 `P₀ … Pₙ`，De Casteljau 算法重复线性插值：

```text
Pᵢ⁽⁰⁾ = Pᵢ
Pᵢ⁽ʳ⁾(t) = (1-t) Pᵢ⁽ʳ⁻¹⁾(t) + t Pᵢ₊₁⁽ʳ⁻¹⁾(t)
B(t) = P₀⁽ⁿ⁾(t), 0 ≤ t ≤ 1
```

程序对 `t = 0, 1/N, …, 1` 采样。`N` 是线段数，结果有 `N+1` 个点。采样点太疏会使前视交点与曲率估算不稳定；点太密会增加搜索开销。`setPath()` 会删除连续重复点，避免零长度线段除零。

## 2. 路径进度与前视点

从上一轮最近线段开始，检查有限个后续线段，找当前点在各线段上的最近投影；路径累计弧长 `s` 只允许向前推进，减少自交路径回跳。

前视半径随实测平移速度 `|v|` 改变：

```text
L = clamp(lookahead + lookahead_speed_gain × |v|,
          min_lookahead, max_lookahead)
```

线段 `A + t(B-A)` 与以当前位置 `C` 为圆心、`L` 为半径的圆相交时：

```text
|A + t(B-A) - C|² = L²,  0 ≤ t ≤ 1
```

把它展开为一元二次方程，取搜索范围内**弧长最靠前且不落后于已有进度**的交点。若无有效交点，则从最近投影点沿路径弧长前进 `L`；超过路径末端时直接取终点。搜索范围由 `nearest_search_segments`、`lookahead_search_segments` 限制。

## 3. 二维平移与限速

先计算 `d = 前视点 - 当前点`，再归一化为方向。前视方向只决定**全局 X/Y 平移**；车头朝向另算，因此允许边沿曲线运动边旋转。终点附近，方向用 smoothstep 逐渐从前视点过渡到终点方向。

用前视线段附近三个采样点 `A, B, C` 估计有符号曲率：

```text
κ = 2 × cross(B-A, C-A) / (|B-A| × |C-B| × |C-A|)
v_curve = clamp(max_speed - curvature_speed_gain × |κ|,
                min_curve_speed, max_speed)
```

到终点距离 `D` 小于 `slowdown_distance` 时，再用归一化的 smoothstep 生成终点速度上限：

```text
q = clamp((D - position_tolerance) /
          (slowdown_distance - position_tolerance), 0, 1)
smoothstep(q) = 3q² - 2q³
v_goal = max_speed × smoothstep(q)
v = min(v_curve, v_goal)
velocity = v × normalize(d)
```

这里的 `v` 是**期望速度大小**，不是由实际制动能力推导出的安全速度。实体平台如果存在延迟或惯性，必须在低层另加加速度限制、速度跟踪和制动验证。

## 4. 独立朝向控制

首次 `update()` 的实际朝向作为起始角。默认按最后 `heading_blend_distance` 的弧长过渡到目标角；调用 `setHeadingTurnRange(start, end)` 后改为指定采样点区间。过渡比例使用 smoothstep，角度差限制在 `[-180°, 180°]`，优先走较短旋转方向。

```text
desired_heading = initial_heading + smoothstep(progress) × shortest_angle_delta
heading_error = shortest_angle(desired_heading - measured_heading)
omega = clamp(heading_kp × heading_error, -omega_max, omega_max)
```

`omega` 是通用的角速度建议值，不是电机 PWM 或某个底盘的旋转百分比。转向区间到达终点后，期望角维持目标角。

## 5. 终点条件

当直线距离 `D ≤ position_tolerance` 时，平移指令为零。只有同时满足以下条件，`finished` 才为真：

- 位置在容差内；
- 实际平移速度不高于 `stop_speed_tolerance`；
- 朝向误差不大于 `heading_tolerance_deg`。

这不是机械制动实现。若车辆冲出容差区，下一轮会继续产生追踪指令；具体如何刹车、是否在终点锁存、是否超时退出，应由应用层根据硬件行为决定。

## 6. 与原实机工程的关系

原工程还包含速度 PID、终点位置 PID 接管、电机指令、惯导和码盘。它们属于平台控制层，不在本仓库内。本仓库对输入输出采用纯数据接口：`Pose`、`actual_velocity`、`target_heading_deg` → `Command`，因此可用于仿真、不同全向底盘，或单纯研究几何路径跟踪。此独立版并不保证与原工程逐周期数值完全相同。
