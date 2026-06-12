%% =========================================================================
%  MPC控制器 - Gain Scheduling MPC
%  基于AMESim线性化模型，支持多工作点调度
%  =========================================================================
%  使用方法：
%    1. 先运行 mpc_setup 初始化MPC对象
%    2. 每个控制周期调用 mpc_step 计算控制量
%  =========================================================================

function mpc_obj = mpc_controller(sys_ss, cfg)
%  输入：sys_ss - 降阶后的状态空间模型 (ss对象)
%        cfg    - MPC配置参数
%  输出：mpc_obj - 配置好的MPC对象

%% ======================== 创建MPC对象 =================================
mpc_obj = mpc(sys_ss, cfg.Ts);

%% ======================== 设置时域 ====================================
mpc_obj.PredictionHorizon = cfg.Np;
mpc_obj.ControlHorizon    = cfg.Nc;

%% ======================== 设置MV约束 ==================================
%  压缩机转速
mpc_obj.MV(1).Min         = cfg.mv_min(1);
mpc_obj.MV(1).Max         = cfg.mv_max(1);
mpc_obj.MV(1).RateMin     = -cfg.mv_rate(1);
mpc_obj.MV(1).RateMax     =  cfg.mv_rate(1);

%  冷凝风扇
mpc_obj.MV(2).Min         = cfg.mv_min(2);
mpc_obj.MV(2).Max         = cfg.mv_max(2);
mpc_obj.MV(2).RateMin     = -cfg.mv_rate(2);
mpc_obj.MV(2).RateMax     =  cfg.mv_rate(2);

%  电池水泵
mpc_obj.MV(3).Min         = cfg.mv_min(3);
mpc_obj.MV(3).Max         = cfg.mv_max(3);
mpc_obj.MV(3).RateMin     = -cfg.mv_rate(3);
mpc_obj.MV(3).RateMax     =  cfg.mv_rate(3);

%  电机水泵
mpc_obj.MV(4).Min         = cfg.mv_min(4);
mpc_obj.MV(4).Max         = cfg.mv_max(4);
mpc_obj.MV(4).RateMin     = -cfg.mv_rate(4);
mpc_obj.MV(4).RateMax     =  cfg.mv_rate(4);

%% ======================== 设置OV约束 ==================================
%  电池温度
mpc_obj.OV(1).Min = cfg.ov_min(1);
mpc_obj.OV(1).Max = cfg.ov_max(1);

%  座舱温度
mpc_obj.OV(2).Min = cfg.ov_min(2);
mpc_obj.OV(2).Max = cfg.ov_max(2);

%  逆变器冷却液温度
mpc_obj.OV(3).Min = cfg.ov_min(3);
mpc_obj.OV(3).Max = cfg.ov_max(3);

%% ======================== 设置权重 ====================================
mpc_obj.Weights.OutputVariables      = diag(cfg.Q_base)';
mpc_obj.Weights.ManipulatedVariables = diag(cfg.R_base)';
mpc_obj.Weights.ECR                  = 1e5;  % 约束松弛权重

%% ======================== 设置扰动模型 ================================
%  扰动通过md参数在运行时传入
%  不设置内置扰动模型，使用外部传入

end
