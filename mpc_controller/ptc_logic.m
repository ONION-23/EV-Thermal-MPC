%% =========================================================================
%  PTC逻辑控制
%  电池PTC和座舱PTC的开关控制（迟滞控制，避免频繁开关）
%  =========================================================================

function [ptc_batt_on, ptc_cabin_on] = ptc_logic(meas, cfg)
%  输入：meas - 测量值结构体
%        cfg  - 配置参数
%  输出：ptc_batt_on  - 电池PTC开关 (0/1)
%        ptc_cabin_on - 座舱PTC开关 (0/1)

%% ======================== 电池PTC逻辑 =================================
%  条件：电池温度过低时开启，回升到安全值后关闭
%  迟滞区间：[ptc_batt_on, ptc_batt_off] = [10℃, 15℃]

if meas.T_battery < cfg.ptc_batt_on
    ptc_batt_on = 1;    % 电池太冷，开启加热
elseif meas.T_battery > cfg.ptc_batt_off
    ptc_batt_on = 0;    % 电池温度恢复，关闭
else
    ptc_batt_on = -1;   % 迟滞区间内，保持上一状态（-1表示不变）
end

%% ======================== 座舱PTC逻辑 =================================
%  条件：极寒工况下辅助热泵制热
%  迟滞区间：环境温度 < -10℃ 开启，座舱 > 20℃ 关闭

if meas.T_env < cfg.ptc_cabin_on
    ptc_cabin_on = 1;   % 极寒环境，辅助制热
elseif meas.T_cabin > cfg.ptc_cabin_off
    ptc_cabin_on = 0;   % 座舱足够暖，关闭
else
    ptc_cabin_on = -1;  % 迟滞区间内，保持上一状态
end

end
