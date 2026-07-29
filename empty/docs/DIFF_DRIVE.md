# 四轮差速圆弧控制

## 适用结构与尺寸

本车没有阿克曼转向，M1/M2 为左侧两轮，M3/M4 为右侧两轮。同一侧两轮使用相同 RPM，因此按左右两组轮的差速模型控制。

当前标称几何参数位于 `module/diff_drive/diff_drive.c`：

| 参数 | 数值 | 用途 |
|---|---:|---|
| 左右轮中心距 `trackWidthMm` | 201 mm | 差速圆弧计算直接使用 |
| 前后轮中心距 `wheelBaseMm` | 201 mm | 四轮固定结构会侧滑，供实测校正参考 |
| 轮胎宽度 `tireWidthMm` | 27 mm | 供实测校正参考 |

轮径当前未知；模块以电机 RPM 作为速度输入，不需要轮径。后续若要输入 mm/s，再补测轮径并增加线速度换算接口。

## 圆弧公式与调用

给定小车几何中心到圆心的带符号半径 `R`、左右轮中心距 `W`、车体中心速度对应的 RPM `C`：

```text
leftRpm  = C × (R - W / 2) / R
rightRpm = C × (R + W / 2) / R
```

- `R > 0` 为左转，左侧是内轮；`R < 0` 为右转，右侧是内轮。
- `C > 0` 前进，`C < 0` 倒退。
- `R = 0` 不做圆弧计算，应使用原地转向接口。

任务代码通常只需调用下面这一行；模块会把左 RPM 下发给 M1/M2、右 RPM 下发给 M3/M4，且立即以目标 RPM 执行：

```c
if (DiffDrive_RunRadiusTurn(200, 100)) {
    /* 200 mm 半径左转，中心速度 100 RPM，已立即下发。 */
}
```

`DiffDrive_RunRadiusTurn()` 不会修改给定的中心 RPM：若计算出的任一轮超过 300 RPM，或单侧非零 RPM 低于 5 RPM，函数返回 `false` 且不驱动电机。需要查看或限幅左右 RPM 时，再使用底层的 `DiffDrive_CalcRadiusTurn()`。

## 任务二测试

`DiffDrive_RunRadiusTurn()` 保留为可复用的圆弧运动学接口；题目二已改为正式的 `LINE PID` 循迹，不再调用该接口或保留圆弧测试参数。若后续需要单独验证圆弧，可在专用测试任务中直接调用此接口。

进入后小车立即以目标 RPM 持续圆弧行驶，K4 立即急停并失能。当前 200 mm 左转会得到约左侧 50 RPM、右侧 150 RPM。

## 实车校正与限制

四个固定轮转弯不可避免有轮胎侧滑，所以 201 mm 是标称轮距，不一定等于实际运动学“有效轮距”。如实测在命令 `R_cmd` 下的半径为 `R_measured`，可将 `trackWidthMm` 从当前值改为：

```text
新的有效轮距 = 当前轮距 × R_measured / R_cmd
```

然后重复圆弧测试，直到实测半径接近命令半径。前后轮距和 27 mm 胎宽会影响侧滑，速度越高、半径越小，这种误差通常越明显。

当前电机没有速度/里程编码器反馈，因此这是“立即执行的开环差速”：它不会自动纠正打滑、负载变化或丢步；直接起转可能失步。以后若需要高精度半径，应加入编码器或用 IMU Yaw 做闭环修正。

## 参考实现

- [WPILib Differential Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html)：以左右轮距将车体线速度与角速度换算为左右轮速度。
- [ROS 2 diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)：多轮同侧差速的配置方式、轮距校正参数与侧滑注意事项。
