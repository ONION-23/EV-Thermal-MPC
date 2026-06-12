%% =========================================================================
%  热管理系统模式切换控制器
%  根据环境温度、电池温度、SOC等条件自动切换运行模式
%  =========================================================================
%  输入：传感器测量值 + 配置参数
%  输出：运行模式 + 各执行器开关状态
%  =========================================================================

function mode_info = thermal_mode_switch(meas, cfg)
%  meas.T_env        - 环境温度 [℃]
%  meas.T_battery    - 电池温度 [℃]
%  meas.T_cabin      - 座舱温度 [℃]
%  meas.T_inv_out    - 逆变器冷却液出口温度 [℃]
%  meas.SOC          - 电池SOC [0~1]
%  meas.v_vehicle    - 车速 [km/h]
%
%  mode_info.mode          - 模式编号 (1~9)
%  mode_info.mode_name     - 模式名称
%  mode_info.heat_pump     - 热泵模式: 'off' / 'heating' / 'cooling'
%  mode_info.battery_loop  - 电池回路: 'series' / 'parallel' / 'off'
%  mode_info.motor_loop    - 电机回路: 'AB_heat_recovery' / 'AC_radiator'
%  mode_info.ptc_batt_on   - 电池PTC开关 (0/1)
%  mode_info.ptc_cabin_on  - 座舱PTC开关 (0/1)
%  mode_info.valve_HR      - 余热回收阀 (0/1)

%% ======================== 参数提取 ====================================
T_env     = meas.T_env;
T_batt    = meas.T_battery;
T_cabin   = meas.T_cabin;
T_inv_out = meas.T_inv_out;
SOC       = meas.SOC;

%% ======================== 模式判断 ====================================

% ------ 工况1: 极寒强制制热 (T_env < -10℃) ------
if T_env < -10
    mode = 1;
    mode_name = '极寒强制制热';
    heat_pump    = 'heating';
    battery_loop = 'series';      % 串联：电机余热加热电池
    motor_loop   = 'AB_heat_recovery';
    valve_HR     = (T_inv_out > cfg.heat_recovery_Tmin);  % 余热够才开
    [ptc_batt_on, ptc_cabin_on] = ptc_logic(meas, cfg);

% ------ 工况2: 普通制热 (-10℃ ≤ T_env < 0℃) ------
elseif T_env < 0
    mode = 2;
    mode_name = '普通制热';
    heat_pump    = 'heating';
    battery_loop = 'series';
    motor_loop   = 'AB_heat_recovery';
    valve_HR     = (T_inv_out > cfg.heat_recovery_Tmin);
    [ptc_batt_on, ptc_cabin_on] = ptc_logic(meas, cfg);

% ------ 工况3: 过渡制热 (0℃ ≤ T_env < 10℃, 座舱仍需加热) ------
elseif T_env < 10 && T_cabin < 22
    mode = 3;
    mode_name = '过渡制热';
    heat_pump    = 'heating';
    battery_loop = 'series';
    motor_loop   = 'AB_heat_recovery';
    valve_HR     = (T_inv_out > cfg.heat_recovery_Tmin);
    [ptc_batt_on, ptc_cabin_on] = ptc_logic(meas, cfg);

% ------ 工况4: 低温电池加热 (0℃ ≤ T_env < 10℃, 座舱不需加热) ------
elseif T_env < 10 && T_batt < cfg.T_batt_cold
    mode = 4;
    mode_name = '低温电池加热';
    heat_pump    = 'off';         % 座舱不需要加热
    battery_loop = 'series';
    motor_loop   = 'AB_heat_recovery';
    valve_HR     = (T_inv_out > cfg.heat_recovery_Tmin);
    [ptc_batt_on, ptc_cabin_on] = ptc_logic(meas, cfg);

% ------ 工况5: 温和模式 (10℃ ≤ T_env < 20℃) ------
elseif T_env < cfg.T_env_hot
    mode = 5;
    mode_name = '温和模式';
    heat_pump    = 'off';
    battery_loop = 'parallel';    % 并联：独立散热
    motor_loop   = 'AC_radiator'; % 常规散热
    valve_HR     = 0;
    ptc_batt_on  = 0;
    ptc_cabin_on = 0;

% ------ 工况6: 常规制冷 (T_env ≥ 20℃, T_batt < 32℃) ------
elseif T_env >= cfg.T_env_hot && T_batt < cfg.T_batt_hot
    mode = 6;
    mode_name = '常规制冷（仅座舱）';
    heat_pump    = 'cooling';
    battery_loop = 'parallel';
    motor_loop   = 'AC_radiator';
    valve_HR     = 0;
    ptc_batt_on  = 0;
    ptc_cabin_on = 0;

% ------ 工况7: 座舱+电池同时制冷 (T_env ≥ 20℃, T_batt ≥ 32℃) ------
elseif T_env >= cfg.T_env_hot && T_batt >= cfg.T_batt_hot
    mode = 7;
    mode_name = '座舱+电池联合制冷';
    heat_pump    = 'cooling';
    battery_loop = 'parallel';
    motor_loop   = 'AC_radiator';
    valve_HR     = 0;
    ptc_batt_on  = 0;
    ptc_cabin_on = 0;

% ------ 工况8: 纯电池冷却 (座舱舒适但电池偏热) ------
elseif T_batt >= 30 && T_cabin >= 20 && T_cabin <= 26
    mode = 8;
    mode_name = '纯电池冷却';
    heat_pump    = 'cooling';
    battery_loop = 'parallel';
    motor_loop   = 'AC_radiator';
    valve_HR     = 0;
    ptc_batt_on  = 0;
    ptc_cabin_on = 0;

% ------ 工况9: 待机/关闭 ------
else
    mode = 9;
    mode_name = '待机';
    heat_pump    = 'off';
    battery_loop = 'off';
    motor_loop   = 'AC_radiator';
    valve_HR     = 0;
    ptc_batt_on  = 0;
    ptc_cabin_on = 0;
end

%% ======================== 余热回收逻辑覆盖 ============================
%  无论什么模式，只要满足余热条件就开启余热回收阀
if strcmp(heat_pump, 'heating') && T_inv_out > cfg.heat_recovery_Tmin
    valve_HR = 1;
end

%% ======================== 输出结构体 ==================================
mode_info.mode          = mode;
mode_info.mode_name     = mode_name;
mode_info.heat_pump     = heat_pump;
mode_info.battery_loop  = battery_loop;
mode_info.motor_loop    = motor_loop;
mode_info.valve_HR      = valve_HR;
mode_info.ptc_batt_on   = ptc_batt_on;
mode_info.ptc_cabin_on  = ptc_cabin_on;

end
