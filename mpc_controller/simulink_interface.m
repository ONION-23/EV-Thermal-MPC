%% =========================================================================
%  Simulink接口说明
%  如何将MPC控制器与Simulink EV Thermal Management模型连接
%  =========================================================================

%% ======================== 接口架构 ====================================
%
%  ┌─────────────────────────────────────────────────────┐
%  │                   Simulink模型                       │
%  │  ┌──────────┐    ┌──────────┐    ┌──────────┐      │
%  │  │  AMESim  │───→│ 传感器   │───→│ MATLAB   │      │
%  │  │  联合仿真│    │ 输出     │    │ Function │      │
%  │  └──────────┘    └──────────┘    └────┬─────┘      │
%  │       ↑                               │             │
%  │       │                               ↓             │
%  │  ┌──────────┐    ┌──────────┐    ┌──────────┐      │
%  │  │ 执行器   │←───│ 控制量   │←───│ MPC      │      │
%  │  │ 输入     │    │ 输出     │    │ 控制器   │      │
%  │  └──────────┘    └──────────┘    └──────────┘      │
%  └─────────────────────────────────────────────────────┘

%% ======================== Simulink中需要的信号 ========================
%  从Simulink模型输出到MATLAB（传感器信号）：
%    1. T_battery          - 电池温度 [℃]
%    2. T_cabin            - 座舱温度 [℃]
%    3. T_inv_out          - 逆变器冷却液出口温度 [℃]
%    4. T_env              - 环境温度 [℃]（工况输入）
%    5. v_vehicle          - 车速 [km/h]（工况输入）
%    6. MotorTorque        - 电机扭矩 [Nm]
%    7. SOC                - 电池SOC [0~1]
%
%  从MATLAB输出到Simulink（控制信号）：
%    1. rpm_comp           - 压缩机转速 [rpm]
%    2. fan_cond           - 冷凝风扇百分比 [%]
%    3. pump_battery       - 电池水泵百分比 [%]
%    4. pump_motor         - 电机水泵百分比 [%]
%    5. ptc_batt_on        - 电池PTC开关 (0/1)
%    6. ptc_cabin_on       - 座舱PTC开关 (0/1)
%    7. valve_HR           - 余热回收阀 (0/1)
%    8. mode               - 当前运行模式 (1~9)

%% ======================== MATLAB Function Block 代码 ==================
%  在Simulink中使用 MATLAB Function Block，代码如下：
%
%  function [rpm_comp, fan_cond, pump_battery, pump_motor, ...
%            ptc_batt_on, ptc_cabin_on, valve_HR, mode] = ...
%            thermal_controller(T_battery, T_cabin, T_inv_out, ...
%            T_env, v_vehicle, MotorTorque, SOC)
%
%  % 持久变量（保持状态）
%  persistent cfg sys_models mv_prev initialized
%  if isempty(initialized)
%      cfg = mpc_config();
%      load('thermal_sys_models.mat', 'sys_models');
%      for i = 1:length(sys_models)
%          sys_models(i).mpc = mpc_controller(sys_models(i).sys, cfg);
%      end
%      mv_prev = [3000; 50; 50; 50];
%      initialized = true;
%  end
%
%  % 构建测量结构体
%  meas.T_env = T_env;
%  meas.T_battery = T_battery;
%  meas.T_cabin = T_cabin;
%  meas.T_inv_out = T_inv_out;
%  meas.SOC = SOC;
%  meas.v_vehicle = v_vehicle;
%  meas.MotorTorque = MotorTorque;
%
%  % 模式切换
%  mode_info = thermal_mode_switch(meas, cfg);
%
%  % PTC逻辑
%  [ptc_b, ptc_c] = ptc_logic(meas, cfg);
%  if ptc_b == -1, ptc_b = ptc_batt_on; end
%  if ptc_c == -1, ptc_c = ptc_cabin_on; end
%
%  % 权重调度
%  [Q, R, ~] = weight_scheduler(meas, cfg);
%
%  % 选择MPC
%  temps = [sys_models.T_env];
%  [~, idx] = min(abs(temps - T_env));
%  mpc_obj = sys_models(idx).mpc;
%  mpc_obj.Weights.OutputVariables = diag(Q)';
%  mpc_obj.Weights.ManipulatedVariables = diag(R)';
%
%  % MPC求解
%  yref = cfg.ov_ref';
%  md = [T_env; v_vehicle; MotorTorque; SOC];
%  ymeas = [T_battery; T_cabin; T_inv_out];
%  mv = mpcmove(mpc_obj, mpcstate(mpc_obj), ymeas, yref, md);
%  mv = max(min(mv, cfg.mv_max), cfg.mv_min);
%  mv_prev = mv;
%
%  % 输出
%  rpm_comp = mv(1);
%  fan_cond = mv(2);
%  pump_battery = mv(3);
%  pump_motor = mv(4);
%  ptc_batt_on = ptc_b;
%  ptc_cabin_on = ptc_c;
%  valve_HR = mode_info.valve_HR;
%  mode = mode_info.mode;
%  end

%% ======================== Simulink模型连接步骤 ========================
%  1. 打开 Simulink EV Thermal Management With Heat Pump Demo
%     >> open_system('ev_thermal_management_with_heat_pump')
%
%  2. 添加 MATLAB Function Block：
%     - 从 Simulink → User-Defined Functions 拖入
%     - 双击编辑，粘贴上面的函数代码
%
%  3. 连接输入信号：
%     - 从模型中提取7个传感器信号连接到Function Block输入
%
%  4. 连接输出信号：
%     - Function Block的8个输出连接到对应的执行器输入
%     - 需要替换原有的控制逻辑（Rule-based）
%
%  5. 设置仿真步长：
%     - Solver → Fixed-step → 5s（与MPC采样时间一致）
%
%  6. 运行仿真

fprintf('Simulink接口说明已显示，请参考上述步骤连接模型\n');
