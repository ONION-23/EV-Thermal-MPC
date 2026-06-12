---
name: ev-thermal-mpc-project
description: 纯电汽车热管理系统MPC控制毕设项目 - 完整技术方案
metadata: 
  node_type: memory
  type: project
  originSessionId: a1185ede-8aaf-4edf-b81c-a80970c2760f
---

## 研究目标
AMESim整车热管理模型 + Simulink EV Thermal Management Demo + 实车数据 + Sobol敏感性分析
三目标优化：电池温度控制 + 座舱舒适性 + 整车热管理能耗

## 热管理架构
- **冷媒回路**：热泵系统（制热/制冷模式）
- **电池冷却回路**：串联模式（低温/制热/余热）+ 并联模式（高温/制冷）
- **电机冷却回路**：AB路径（余热回收，Heating + T>35℃）+ AC路径（Radiator常规散热）
- **PTC**：电池PTC（逻辑控制）+ 座舱PTC（极寒辅助，逻辑控制）

## Sobol分析结论
- 压缩机：全温域最高敏感度 → 必须作为MPC控制量
- 鼓风机：排名第二 → 可作为MPC控制量或PID
- PTC：敏感度低但能耗大 → 逻辑控制
- 电池初温：仅-10℃附近敏感 → SOC调度+权重调度

## MPC设计
- **MV（操纵变量）**：[rpm_comp, fan_cond, pump_battery, pump_motor] 共4个
- **OV（输出变量）**：[T_battery, T_cabin, T_coolant_inverter_out]
- **MD（扰动变量）**：[T_env, v_vehicle, MotorTorque, SOC] 共4个（SOC同时影响电池内阻和发热量）
- **权重调度变量**：ρ = [SOC, T_env]

### MV约束
| 变量 | 最小值 | 最大值 | 单位 | 变化率限制 |
|------|--------|--------|------|-----------|
| rpm_comp | 1000 | 6000 | rpm | ±500 rpm/s |
| fan_cond | 0 | 100 | % | ±20 %/s |
| pump_battery | 10 | 100 | % | ±20 %/s |
| pump_motor | 10 | 100 | % | ±20 %/s |

### OV约束
| 变量 | 最小值 | 最大值 | 单位 |
|------|--------|--------|------|
| T_battery | 25 | 35 | ℃ |
| T_cabin | 20 | 26 | ℃ |
| T_coolant_inverter_out | - | 65 | ℃ |

### MPC参数
- 采样时间 Ts = 5 s
- 预测时域 Np = 40 步（200s）
- 控制时域 Nc = 8 步（40s）
- 权重：Q = diag([10, 8, 2]), R = diag([0.1, 0.1, 0.1, 0.1])

## 逻辑控制（不作为MPC变量）
- 电池PTC：T_battery<10℃开，>15℃关
- 座舱PTC：T_env<-10℃开，T_cabin>20℃关
- 余热回收阀：Heating + T_coolant_inverter_out>35℃开
- 串并联切换阀：逻辑控制

## SOC处理（双重身份）
- **作为扰动变量（MD）**：SOC影响电池内阻R(SOC)→发热量Q=I²·R(SOC)变化，MPC模型需要感知
- **作为权重调度变量（ρ）**：SOC<30%能耗优先，SOC>70%舒适性优先
- 不作为状态量/输出量
- Q = f(SOC, T_env) 动态调整MPC权重

## 技术路线（主线）
1. AMESim建立工作点（T_env=-20℃, SOC=50%, v=60km/h, MotorTorque=80Nm）→ 稳态
2. AMESim Linear Analysis → 直接获得A,B,C,D
3. MATLAB降阶：sys=ss(A,B,C,D) → hsvd(sys) → balred(sys,8) → 6~10阶模型
4. 建立多工作点：-20℃, -10℃, 25℃, 35℃
5. Gain Scheduling MPC，调度变量ρ=[SOC, T_env]
6. 备用方案：N4SID辨识（线性化失败时用）

## 论文第五章结构
- 5.1 热管理系统控制架构分析
- 5.2 MPC控制问题描述（状态/输入/输出/约束）
- 5.3 AMESim工作点线性化（A,B,C,D提取）
- 5.4 模型降阶（平衡截断+误差验证）
- 5.5 基于SOC和环境温度的权重调度MPC
- 5.6 仿真验证（Rule-based vs MPC，评价指标：电池温度误差/座舱舒适度/总能耗/压缩机能耗/PTC能耗）

## GitHub仓库
- 地址：https://github.com/ONION-23/EV-Thermal-MPC
- 本地路径：C:\Users\岳彩腾\Desktop\test
- 已推送文件：mpc_config.m, thermal_mode_switch.m, ptc_logic.m, weight_scheduler.m, mpc_controller.m, main_controller.m, simulink_interface.m

## 当前进展
- [x] Git + GitHub 配置完成
- [x] MPC变量定义（MV/OV/MD/ρ）
- [x] MPC约束范围填写（占位值，待用户确认）
- [x] 逻辑切换框架（9种模式）
- [x] PTC迟滞控制逻辑
- [x] 权重调度器（SOC/T_env）
- [x] MPC控制器框架代码
- [x] Simulink接口说明
- [ ] 用户确认/修改约束数值
- [ ] AMESim线性化 → 获取A,B,C,D
- [ ] MATLAB模型降阶
- [ ] Simulink联合仿真调试
- [ ] Rule-based对比仿真

**Why:** 这是毕设第五章的核心技术方案，需在开发过程中持续参考。
**How to apply:** 所有MPC开发、代码编写、Simulink建模都基于此方案执行。
