# 贝塞尔曲线与二维纯追踪（C++）

一个不依赖机器人品牌、驱动库或特定底盘的算法示例：先用贝塞尔曲线生成离散路径，再根据当前位姿计算二维平移速度与目标角速度。**平移方向与机体朝向分别计算**，适合研究可独立平移、旋转的全向运动平台。

本仓库是从一个实机工程中整理出的**独立参考实现**，不是原工程的原样复制；比赛路线、设备驱动、速度 PID 和终点 `Goto` 接管均未包含。它输出运动指令，不负责读取定位传感器、控制电机或保证物理车辆能准确停住。

## 它做了什么

1. 对控制点执行 De Casteljau 插值，得到 `segments + 1` 个路径采样点。
2. 在当前路径进度附近找最近线段，并将当前位置投影到线段上。
3. 用当前位置为圆心的前视圆与后续线段求交，选取前方交点；没有交点时沿路径弧长回退寻找。
4. 将当前位置到前视点的方向归一化，乘以由曲率和终点距离限制后的目标速度。
5. 航向角独立插值：默认在路径末段转向，也可指定两个**采样点下标**，只在该区间平滑转向。
6. 进入终点位置容差后给出零平移指令；位置、朝向和实际速度均满足条件时报告 `finished`。

推导、公式和边界情况见 [算法说明](docs/algorithm.md)。

## 快速运行

需要支持 C++11 的编译器和 CMake 3.10+：

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

示例程序位于 `examples/main.cpp`，构建后运行 `example`（Windows 的 Visual Studio 多配置生成器下通常为 `build/Debug/example.exe`）。它按曲线上的采样点打印控制器输出，**不是物理运动仿真**。

## 最小用法

```cpp
#include "bezier.h"
#include "pure_pursuit.h"

std::vector<pp::Point> controls = {
    pp::Point(0, 0), pp::Point(40, 100), pp::Point(100, 100)
};
pp::PurePursuit tracker;
tracker.setPath(pp::sampleBezier(controls, 100));

// 可选：只在第 40 至第 70 个采样点之间转向。
tracker.setHeadingTurnRange(40, 70);

// 在自己的固定周期控制循环中反复调用。
pp::Pose pose(pp::Point(10, 20), 0);       // 位置、当前朝向
pp::Point measured_velocity(5, 0);       // 全局坐标下的实际速度
pp::Command cmd = tracker.update(pose, measured_velocity, 90);
// cmd.velocity.x / y、cmd.angular_speed_deg_s 交给你的平台适配层。
```

不要把示例中固定写死的 `pose` 当作真实控制循环；每轮必须传入最新测量值。新路线调用 `setPath()`；若要在同一路线上重新追踪，调用 `reset()`。

## 坐标与单位

| 量 | 约定 |
| --- | --- |
| 路径与位置 | 任意一致的长度单位，推荐 cm |
| 平移速度 | 长度单位/秒；若位置用 cm，速度就是 cm/s |
| 朝向 | 全局 `+X` 为 `0°`，逆时针为正 |
| 角速度 | `°/s`，逆时针为正 |
| 曲率 | 长度单位的倒数，例如 `1/cm` |

`Command.velocity` 是**全局坐标**中的期望平移速度，不是轮速；不同底盘应自行完成坐标变换、逆运动学和低层速度控制。原实机工程采用另一套朝向与速度定义，此处特意改为常见数学坐标约定，避免将其硬件符号约定带入通用算法。

## 主要参数

参数在 `include/pure_pursuit.h` 的 `pp::Config` 中。`lookahead` 越大通常越平滑，但可能更容易切弯；`lookahead_speed_gain` 使前视距离随实测速度增加。`curvature_speed_gain` 控制弯道降速，`slowdown_distance` 控制终点减速开始距离，`heading_blend_distance` 控制默认转向区间长度。`position_tolerance`、`heading_tolerance_deg` 和 `stop_speed_tolerance` 共同决定完成条件。

这些默认值仅用于演示，不能直接当成某台实体机器的安全参数。采样越密，同样的下标区间表示的实际距离可能越短；若需要稳定的物理转向区间，建议把起止点按路径弧长转换成采样下标。

## 已知边界

- 这是**几何跟踪与目标速度规划**，没有电机动力学、闭环速度 PID、加速度/制动距离模型、碰撞检测或障碍物避让。
- 终点减速只依据当前位置到终点的距离；实际制动能力不足时仍可能超调。`finished` 只是判定信号，不能代替底盘制动。
- 前视与最近点搜索采用有界、单向路径进度；自交路径或大幅脱离路线时，仍需结合现场调节搜索范围或重新规划。
- 调用方负责传入真实且同单位的实际速度。角速度是建议值，执行器的饱和、加速度限制与安全停机应由适配层负责。

## 目录

```text
include/   公开头文件与数据类型
src/       贝塞尔和纯追踪实现
examples/  无硬件示例
tests/     基本几何、转向和终点判定测试
docs/      原理说明
```

## 许可与来源

本算法项目采用 [MIT License](LICENSE)，版权署名为“崔晓”。发布前请核对这个署名是否与你希望对外显示的姓名一致。不要把原比赛工程的 Git 历史直接推送到此目录。

具体发布步骤见 [PUBLISHING.md](PUBLISHING.md)。

