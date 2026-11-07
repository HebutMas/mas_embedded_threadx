# 算法来源
1. [RoboMaster机器人姿态解算方案开源](https://zhuanlan.zhihu.com/p/540676773)
2. [WangHongxi2001/RoboMaster-C-Board-INS-Example](https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example)
3. [HNUYueLuRM/basic_framework](https://gitee.com/hnuyuelurm/basic_framework)

# IMU输入及控制反馈

- `Module_INS_Init()` 沿用模块初始化入口；先完成 BMI088 初始化/标定，再创建 INS 线程。
- INS 更新链路：读取 BMI088 → 安装/尺度修正 → 可选三轴 gyro/acc 陷波 → EKF 加速度低通 → 姿态与在线零偏估计。
- EKF 加速度低通时间常数为 `0.0085 s`；在向量归一化之前滤波，防止直接用高频振动判断重力方向。`Ins_t.AccelLPF` 仍用于去重力后的运动加速度，二者作用不同。
- `INS_NOTCH_FREQ_HZ` 默认 `0`（关闭），`INS_NOTCH_BANDWIDTH_HZ` 默认 `15 Hz`；由 `modules/module_config.cmake` 提供默认值，机器人 `robot.cmake` 覆盖。sentry 云台板按当前 6800 rpm 的 CSV 实测启用 `115 Hz` 陷波，底盘板及其他机器人默认不启用。
- 陷波采用按实际 `ins.dt` 更新的二阶 TPT 积分器，首次输入按直流状态初始化；无效时间步或接近奈奎斯特限制时重置状态并透传当次输入。该滤波不能替代采样前抗混叠或电机动平衡。
- `Ins_t.gyro[3]` 单位为 `rad/s`，复制 EKF 本轮预测所用的角速度，已包含安装修正、可选陷波和在线零偏补偿。sentry pitch LQR 使用 `ins->gyro[0]`，避免角度与速度使用不同的零偏处理。BMI088 的 `gyro/acc` 保留陷波前数据，便于原始振动记录；`QEKF_INS.Accel` 为陷波、低通后的加速度。
- `StableFlag` 用角速度模长及加速度模长判断卡方失配恢复条件，不用机体系 `sqrt(ax²+ay²)` 判静止，因为倾斜云台的 XY 轴也含重力。保留原角速度门限、加速度模长门限、卡方拒绝及恢复计数。

# 验证与记录

- 改动后先做目标板构建，再在底座尽量固定时比较摩擦轮关闭、开启和关闭后的保持效果。软件回放无法证明实机闭环稳定。
- 同步记录 `ins->euler_rad[1]`、`ins->gyro[0]`、pitch 编码器、BMI088 三轴 acc、`QEKF_INS.GyroBias[0]`、`QEKF_INS.ChiSquare_Data[0]`、`QEKF_INS.StableFlag`、`QEKF_INS.ErrorCount`、`ins->dt` 及摩擦轮实际开关状态。编码器报告角度需先核对电机位置量程与机械传动比例。
