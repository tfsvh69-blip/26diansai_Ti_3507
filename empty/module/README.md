# module

用于放置电机、传感器、遥控器、OLED、通信协议解析等可复用模块。

模块可以依赖 `bsp/` 提供的硬件能力，但应避免直接依赖 `app/` 的业务任务。

当前 `imu/` 封装 ATK-MS6DSV/LSM6DSV16X 初始化、SFLP FIFO 读取和四元数转欧拉角；`oled/` 封装 OLED 显示驱动。
