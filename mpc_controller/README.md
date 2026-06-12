# 纯电汽车热管理系统 MPC 控制器

## 项目概述

基于 Gain Scheduling MPC 的纯电汽车热管理系统控制器，集成模式切换逻辑、PTC控制、权重调度等功能。

## 文件说明

| 文件 | 功能 |
|------|------|
| `mpc_config.m` | MPC参数配置（约束、权重、阈值等） |
| `thermal_mode_switch.m` | 热管理运行模式切换逻辑（9种模式） |
| `ptc_logic.m` | PTC开关逻辑控制（迟滞控制） |
| `weight_scheduler.m` | MPC权重动态调度（基于SOC和环境温度） |
| `mpc_controller.m` | MPC控制器（基于MATLAB MPC Toolbox） |
| `main_controller.m` | 主控制器（完整仿真流程） |
| `simulink_interface.m` | Simulink接口说明和连接步骤 |

## 系统架构

```
传感器信号 → 模式切换 → PTC逻辑 → 权重调度 → MPC求解 → 执行器
                    ↑                              ↑
              逻辑控制（PTC/阀门）         Gain Scheduling
```

## 运行模式

| 模式 | 条件 | 热泵 | 电池回路 | PTC |
|------|------|------|---------|-----|
| 1. 极寒强制制热 | T_env < -10℃ | 制热 | 串联 | 电池+座舱 |
| 2. 普通制热 | -10℃ ≤ T_env < 0℃ | 制热 | 串联 | 按逻辑 |
| 3. 过渡制热 | 0℃ ≤ T_env < 10℃, 座舱需加热 | 制热 | 串联 | 按逻辑 |
| 4. 低温电池加热 | 0℃ ≤ T_env < 10℃, 座舱舒适 | 关闭 | 串联 | 按逻辑 |
| 5. 温和模式 | 10℃ ≤ T_env < 20℃ | 关闭 | 并联 | 关闭 |
| 6. 常规制冷 | T_env ≥ 20℃, 电池<32℃ | 制冷 | 并联 | 关闭 |
| 7. 联合制冷 | T_env ≥ 20℃, 电池≥32℃ | 制冷 | 并联 | 关闭 |
| 8. 纯电池冷却 | 电池≥30℃, 座舱舒适 | 制冷 | 并联 | 关闭 |
| 9. 待机 | 其他 | 关闭 | 关闭 | 关闭 |

## MPC变量

```
MV = [rpm_comp, fan_cond, pump_battery, pump_motor]   # 4个操纵变量
OV = [T_battery, T_cabin, T_inv_out]                  # 3个输出变量
MD = [T_env, v_vehicle, MotorTorque, SOC]             # 4个扰动变量
ρ  = [SOC, T_env]                                     # 权重调度变量
```

## 使用步骤

### 1. 独立运行（MATLAB脚本）
```matlab
cd mpc_controller
main_controller    % 运行完整仿真
```

### 2. 与Simulink联合使用
1. 打开 Simulink EV Thermal Management 模型
2. 添加 MATLAB Function Block
3. 参考 `simulink_interface.m` 连接信号
4. 设置仿真步长为 5s
5. 运行仿真

### 3. 使用真实AMESim模型
1. 在 AMESim 中完成线性化分析
2. 导出 A, B, C, D 矩阵
3. 在 MATLAB 中降阶：`sysr = balred(ss(A,B,C,D), 8)`
4. 保存到 `thermal_sys_models.mat`
5. 运行 `main_controller`

## 参数修改

修改 `mpc_config.m` 中的数值：
- 约束范围：`mv_min`, `mv_max`, `ov_min`, `ov_max`
- MPC参数：`Ts`, `Np`, `Nc`
- PTC阈值：`ptc_batt_on`, `ptc_batt_off`
- 模式切换温度：`T_env_cold`, `T_env_hot`

## 依赖

- MATLAB R2020b+
- Model Predictive Control Toolbox
- Control System Toolbox
- Simulink（与Simulink联合使用时）
