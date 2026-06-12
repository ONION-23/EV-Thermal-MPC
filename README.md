# 纯电汽车热管理系统 MPC 控制

基于 Gain Scheduling MPC 的纯电汽车热管理系统控制器，集成模式切换逻辑、PTC控制、权重调度等功能。

## 项目结构

```
├── mpc_controller/          # MPC控制器核心代码
│   ├── mpc_config.m         # 参数配置
│   ├── thermal_mode_switch.m # 9种运行模式切换
│   ├── ptc_logic.m          # PTC逻辑控制
│   ├── weight_scheduler.m   # 权重调度器
│   ├── mpc_controller.m     # MPC控制器
│   ├── main_controller.m    # 主仿真脚本
│   └── simulink_interface.m # Simulink接口说明
├── simulink/                # Simulink模型文件（待添加）
├── amesim/                  # AMESim模型文件（待添加）
└── docs/                    # 文档资料
```

## 研究目标

- 电池温度控制
- 座舱舒适性
- 整车热管理能耗

## MPC 变量

| 类型 | 变量 |
|------|------|
| MV (操纵) | 压缩机转速, 冷凝风扇, 电池泵, 电机泵 |
| OV (输出) | 电池温度, 座舱温度, 逆变器冷却液温度 |
| MD (扰动) | 环境温度, 车速, 电机扭矩, SOC |

## 依赖

- MATLAB R2020b+
- Model Predictive Control Toolbox
- Control System Toolbox
- Simulink（联合仿真时）
