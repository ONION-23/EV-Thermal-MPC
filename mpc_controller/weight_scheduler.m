%% =========================================================================
%  MPC权重调度器
%  根据SOC和环境温度动态调整MPC权重矩阵
%  =========================================================================
%  调度逻辑：
%    SOC < 30%  → 能耗优先（降低舒适性权重，提高能耗权重）
%    SOC > 70%  → 舒适优先（提高舒适性权重）
%    T_env极低  → 电池保温优先（提高电池温度权重）
%    T_env极高  → 电池散热优先（提高电池温度权重）
%  =========================================================================

function [Q, R, dQ] = weight_scheduler(meas, cfg)
%  输入：meas - 测量值结构体（需含SOC, T_env）
%        cfg  - 配置参数
%  输出：Q  - 输出跟踪权重 [3×3]
%        R  - 控制量变化权重 [4×4]
%        dQ - 输出变化率权重 [3×3]

SOC    = meas.SOC;
T_env  = meas.T_env;

%% ======================== SOC调度因子 ================================
%  SOC越低 → 越关注能耗 → 降低输出跟踪权重
if SOC < cfg.soc_low
    % SOC<30%: 能耗优先，降低舒适性权重
    soc_factor_batt  = 0.6;    % 电池温度跟踪权重降低
    soc_factor_cabin = 0.5;    % 座舱温度跟踪权重降低
    soc_factor_R     = 2.0;    % 控制量变化惩罚增大（减少能耗）
elseif SOC > cfg.soc_high
    % SOC>70%: 舒适优先，提高舒适性权重
    soc_factor_batt  = 1.2;
    soc_factor_cabin = 1.3;
    soc_factor_R     = 0.8;
else
    % 30%≤SOC≤70%: 均衡模式
    soc_factor_batt  = 1.0;
    soc_factor_cabin = 1.0;
    soc_factor_R     = 1.0;
end

%% ======================== 环境温度调度因子 =============================
%  极端温度 → 提高电池温度跟踪权重
if T_env < -10
    % 极寒：电池保温更重要
    env_factor_batt = 1.5;
elseif T_env > 35
    % 极热：电池散热更重要
    env_factor_batt = 1.5;
else
    env_factor_batt = 1.0;
end

%% ======================== 合成权重矩阵 ================================
Q = cfg.Q_base;
Q(1,1) = cfg.Q_base(1,1) * soc_factor_batt * env_factor_batt;  % T_battery
Q(2,2) = cfg.Q_base(2,2) * soc_factor_cabin;                   % T_cabin
Q(3,3) = cfg.Q_base(3,3);                                       % T_inv（固定）

R = cfg.R_base * soc_factor_R;

dQ = cfg.dQ_base;

end
