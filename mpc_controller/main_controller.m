%% =========================================================================
%  主控制器 - 纯电汽车热管理系统
%  集成模式切换 + PTC逻辑 + MPC控制 + 权重调度
%  =========================================================================
%  使用方法：
%    1. 准备好AMESim线性化模型 sys_models（结构体数组）
%    2. 运行本脚本
%  =========================================================================
clear; clc; close all;

%% ======================== 加载配置 ====================================
cfg = mpc_config();
fprintf('配置加载完成\n');

%% ======================== 加载线性化模型 ==============================
%  从MAT文件加载（由AMESim导出后在MATLAB中处理）
%  sys_models(i).T_env  - 工作点环境温度
%  sys_models(i).sys    - 降阶后的ss模型
%  sys_models(i).mpc    - 对应的MPC对象
%
%  如果还没有线性化模型，先用占位代码，后续替换

try
    load('thermal_sys_models.mat', 'sys_models');
    fprintf('已加载 %d 个工作点模型\n', length(sys_models));
catch
    fprintf('警告：未找到线性化模型文件，使用演示模式\n');
    sys_models = create_demo_models(cfg);
end

%% ======================== 初始化MPC对象 ===============================
for i = 1:length(sys_models)
    sys_models(i).mpc = mpc_controller(sys_models(i).sys, cfg);
    fprintf('工作点 %d (T_env=%d℃): MPC初始化完成\n', i, sys_models(i).T_env);
end

%% ======================== 仿真参数 ====================================
T_sim  = 3600;      % 仿真时长 [s]（1小时）
dt     = cfg.Ts;     % 控制步长 [s]
N_step = floor(T_sim / dt);  % 总步数

%% ======================== 初始状态 ====================================
%  测量值初始状态
meas.T_env     = 25;       % 环境温度 [℃]
meas.T_battery = 28;       % 电池初始温度 [℃]
meas.T_cabin   = 25;       % 座舱初始温度 [℃]
meas.T_inv_out = 30;       % 逆变器冷却液出口温度 [℃]
meas.SOC       = 0.60;     % 初始SOC
meas.v_vehicle = 60;       % 车速 [km/h]
meas.MotorTorque = 80;     % 电机扭矩 [Nm]

%  MPC上一步控制量（用于变化率约束）
mv_prev = [3000; 50; 50; 50];

%% ======================== 数据记录 ====================================
log.time       = (0:N_step-1) * dt;
log.T_battery  = zeros(1, N_step);
log.T_cabin    = zeros(1, N_step);
log.T_inv_out  = zeros(1, N_step);
log.mv         = zeros(4, N_step);
log.mode       = zeros(1, N_step);
log.ptc_batt   = zeros(1, N_step);
log.ptc_cabin  = zeros(1, N_step);

%% ======================== 主仿真循环 ==================================
fprintf('\n========== 开始仿真 ==========\n');

for k = 1:N_step
    %% ---- Step 1: 读取传感器数据（实际使用时替换为Simulink接口） ----
    %  这里简化为直接使用meas结构体
    %  实际使用时：meas = read_from_simulink();

    %% ---- Step 2: 模式切换判断 ----
    mode_info = thermal_mode_switch(meas, cfg);

    %% ---- Step 3: PTC逻辑控制 ----
    %  处理迟滞逻辑（-1表示保持上一状态）
    [ptc_batt, ptc_cabin] = ptc_logic(meas, cfg);
    if ptc_batt == -1, ptc_batt = log.ptc_batt(max(k-1,1)); end
    if ptc_cabin == -1, ptc_cabin = log.ptc_cabin(max(k-1,1)); end

    %% ---- Step 4: 权重调度 ----
    [Q, R, dQ] = weight_scheduler(meas, cfg);

    %% ---- Step 5: 选择对应工作点的MPC ----
    mpc_idx = select_operating_point(meas.T_env, sys_models);
    mpc_obj = sys_models(mpc_idx).mpc;

    %% ---- Step 6: 更新MPC权重 ----
    mpc_obj.Weights.OutputVariables = diag(Q)';
    mpc_obj.Weights.ManipulatedVariables = diag(R)';

    %% ---- Step 7: 构建MPC输入 ----
    %  参考值（目标温度，可动态调整）
    yref = cfg.ov_ref';

    %  扰动变量
    md = [meas.T_env; meas.v_vehicle; meas.MotorTorque; meas.SOC];

    %  当前输出测量值
    ymeas = [meas.T_battery; meas.T_cabin; meas.T_inv_out];

    %% ---- Step 8: MPC求解 ----
    try
        %  设置上一步控制量（用于变化率约束）
        mpcstate_obj = mpcstate(mpc_obj);
        mpcstate_obj.LastMove = mv_prev;

        %  计算最优控制量
        mv = mpcmove(mpc_obj, mpcstate_obj, ymeas, yref, md);

        %  应用变化率限制
        mv = apply_rate_limit(mv, mv_prev, cfg.mv_rate, dt);
        mv = max(min(mv, cfg.mv_max), cfg.mv_min);  % 幅值限制

    catch ME
        fprintf('步 %d: MPC求解失败 (%s)，使用上一步控制量\n', k, ME.message);
        mv = mv_prev;
    end

    %% ---- Step 9: 输出到执行器 ----
    %  实际使用时替换为Simulink接口
    %  write_to_simulink(mv, ptc_batt, ptc_cabin, mode_info);
    %  [meas, sim_time] = sim_step();  % 推进Simulink仿真一步

    %% ---- Step 10: 记录数据 ----
    log.T_battery(k) = meas.T_battery;
    log.T_cabin(k)   = meas.T_cabin;
    log.T_inv_out(k) = meas.T_inv_out;
    log.mv(:,k)      = mv;
    log.mode(k)      = mode_info.mode;
    log.ptc_batt(k)  = ptc_batt;
    log.ptc_cabin(k) = ptc_cabin;

    %% ---- Step 11: 更新状态（简化模型，实际由Simulink提供） ----
    mv_prev = mv;
    %  实际使用时不需要这里的简化模型
    meas = simple_thermal_model(meas, mv, ptc_batt, ptc_cabin, mode_info, cfg, dt);

    %% ---- 进度显示 ----
    if mod(k, floor(N_step/10)) == 0
        fprintf('进度: %d%% | 模式: %s | MV: [%.0f, %.0f, %.0f, %.0f]\n', ...
            round(k/N_step*100), mode_info.mode_name, mv(1), mv(2), mv(3), mv(4));
    end
end

fprintf('\n========== 仿真完成 ==========\n');

%% ======================== 结果绘图 ====================================
plot_results(log, cfg);

%% ======================== 保存结果 ====================================
save('simulation_results.mat', 'log', 'cfg');
fprintf('结果已保存至 simulation_results.mat\n');


%% =========================================================================
%  辅助函数
%  =========================================================================

function idx = select_operating_point(T_env, sys_models)
%  根据环境温度选择最近的工作点
    temps = [sys_models.T_env];
    [~, idx] = min(abs(temps - T_env));
end

function mv = apply_rate_limit(mv, mv_prev, rate_max, dt)
%  应用变化率限制
    dv = mv - mv_prev;
    dv_max = rate_max * dt;
    dv = max(min(dv, dv_max), -dv_max);
    mv = mv_prev + dv;
end

function sys_models = create_demo_models(cfg)
%  创建演示用的占位模型（实际使用时替换为AMESim线性化结果）
    fprintf('创建演示模型...\n');

    %  简化的3输入4输出状态空间模型
    %  状态：[T_batt, T_cabin, T_inv, T_wall]
    A = [-0.01  0.005  0.002  0.001;
          0.005 -0.015  0.001  0.003;
          0.002  0.001 -0.02   0.005;
          0.001  0.003  0.005 -0.01];

    B = [-0.005  0       0.001  0;
          0     -0.003   0      0;
          0      0      -0.004  0.001;
          0      0       0      0];

    C = [1 0 0 0;
         0 1 0 0;
         0 0 1 0];

    D = zeros(3, 4);

    %  为每个工作点创建略有不同的模型
    op_temps = cfg.op_temps;
    for i = 1:length(op_temps)
        A_i = A + 0.001 * randn(size(A));  % 简化：实际应从AMESim导出
        sys_models(i).T_env = op_temps(i);
        sys_models(i).sys = ss(A_i, B, C, D);
    end
end

function meas_new = simple_thermal_model(meas, mv, ptc_batt, ptc_cabin, mode_info, cfg, dt)
%  简化的热力学模型（仅用于演示，实际由Simulink/AMESim提供）
    %  简化的一阶惯性模型
    tau_batt = 300;   % 电池热时间常数 [s]
    tau_cabin = 600;  % 座舱热时间常数 [s]
    tau_inv = 200;    % 逆变器热时间常数 [s]

    %  压缩机影响（简化）
    comp_effect = mv(1) / 6000 * 5;  % 最大降温5℃

    %  PTC加热效果
    ptc_effect_batt = ptc_batt * 3;   % PTC最大加热3℃
    ptc_effect_cabin = ptc_cabin * 2; % 座舱PTC最大加热2℃

    %  泵影响
    pump_batt_effect = mv(3) / 100 * 2;
    pump_motor_effect = mv(4) / 100 * 1.5;

    %  环境温度影响
    env_effect_batt = (meas.T_env - meas.T_battery) * 0.01;
    env_effect_cabin = (meas.T_env - meas.T_cabin) * 0.005;

    %  一阶惯性响应
    meas_new = meas;
    if strcmp(mode_info.heat_pump, 'cooling')
        dT_batt = (-comp_effect + env_effect_batt + ptc_effect_batt) * dt / tau_batt;
        dT_cabin = (-comp_effect * 0.5 + env_effect_cabin + ptc_effect_cabin) * dt / tau_cabin;
    elseif strcmp(mode_info.heat_pump, 'heating')
        dT_batt = (comp_effect * 0.3 + env_effect_batt + ptc_effect_batt) * dt / tau_batt;
        dT_cabin = (comp_effect * 0.5 + env_effect_cabin + ptc_effect_cabin) * dt / tau_cabin;
    else
        dT_batt = (env_effect_batt + ptc_effect_batt) * dt / tau_batt;
        dT_cabin = (env_effect_cabin + ptc_effect_cabin) * dt / tau_cabin;
    end

    dT_inv = (pump_motor_effect - 1) * dt / tau_inv;  % 散热

    meas_new.T_battery = meas.T_battery + dT_batt;
    meas_new.T_cabin   = meas.T_cabin + dT_cabin;
    meas_new.T_inv_out = meas.T_inv_out + dT_inv;
end

function plot_results(log, cfg)
%  绘制仿真结果
    figure('Name', '热管理系统MPC仿真结果', 'Position', [100 100 1200 800]);

    %  温度变化
    subplot(3,2,1);
    plot(log.time/60, log.T_battery, 'b-', 'LineWidth', 1.5); hold on;
    yline(cfg.ov_ref(1), 'r--', '目标');
    yline(cfg.ov_min(1), 'g--'); yline(cfg.ov_max(1), 'g--');
    xlabel('时间 [min]'); ylabel('电池温度 [℃]');
    title('电池温度'); grid on; legend('实际', '目标', '约束');

    subplot(3,2,2);
    plot(log.time/60, log.T_cabin, 'r-', 'LineWidth', 1.5); hold on;
    yline(cfg.ov_ref(2), 'r--', '目标');
    yline(cfg.ov_min(2), 'g--'); yline(cfg.ov_max(2), 'g--');
    xlabel('时间 [min]'); ylabel('座舱温度 [℃]');
    title('座舱温度'); grid on; legend('实际', '目标', '约束');

    subplot(3,2,3);
    plot(log.time/60, log.T_inv_out, 'm-', 'LineWidth', 1.5); hold on;
    yline(cfg.ov_max(3), 'g--', '上限');
    xlabel('时间 [min]'); ylabel('逆变器冷却液 [℃]');
    title('逆变器冷却液温度'); grid on;

    %  控制量
    subplot(3,2,4);
    plot(log.time/60, log.mv(1,:), 'LineWidth', 1.5);
    xlabel('时间 [min]'); ylabel('转速 [rpm]');
    title('压缩机转速'); grid on;

    subplot(3,2,5);
    plot(log.time/60, log.mv(2,:), 'LineWidth', 1.5); hold on;
    plot(log.time/60, log.mv(3,:), 'LineWidth', 1.5);
    plot(log.time/60, log.mv(4,:), 'LineWidth', 1.5);
    xlabel('时间 [min]'); ylabel('百分比 [%]');
    title('风扇/水泵'); grid on; legend('冷凝风扇', '电池泵', '电机泵');

    %  运行模式
    subplot(3,2,6);
    stairs(log.time/60, log.mode, 'k-', 'LineWidth', 1.5);
    xlabel('时间 [min]'); ylabel('模式编号');
    title('运行模式'); grid on;
    ylim([0 10]);

    sgtitle('纯电汽车热管理系统 Gain Scheduling MPC 控制效果', 'FontSize', 14);
end
