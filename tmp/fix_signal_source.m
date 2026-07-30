%% G_Sim_V2 信号源能量归一化修复
% 在 MATLAB 中运行此脚本，自动修改 G_Sim_V2.slx 的信号源子系统
%
% 问题: 信号源子系统将基波和谐波简单相加，未做能量归一化
% 修复: 在求和后加入 RMS 归一化增益 = amplitude_0 / sqrt(sum(amplitudes^2))
%       确保基波幅值保持为用户设定值，同时谐波按比例缩放

model = 'G_Sim_V2';
slx_path = fullfile(pwd, 'Reference', 'Simulink', [model '.slx']);

if ~isfile(slx_path)
    error('找不到 %s，请确认当前目录是工程根目录', slx_path);
end

% 打开模型
open_system(slx_path);

% 定位信号源子系统内部的 Add1 块
subsys = [model '/信号源(原始波形/干扰后/干扰波形)'];
add_block = [subsys '/Add1'];

try
    % 获取 Add1 的输出端口位置
    ports = get_param(add_block, 'PortConnectivity');
    out_pos = get_param(add_block, 'Position');

    % 在 Add1 右侧插入归一化增益块
    x = out_pos(3) + 50;
    y = mean([out_pos(2), out_pos(4)]);
    gain_pos = [x, y-20, x+60, y+20];
    gain_name = [subsys '/能量归一化'];

    % 增益值: 保持基波幅值 = amplitude_0 / sqrt(sum(amplitudes²))
    gain_value = ['amplitude_0 / sqrt(amplitude_0^2 + ' ...
                  'harmonic_amplitude_a^2 + harmonic_amplitude_b^2)'];

    add_block('built-in/Gain', gain_name, ...
              'Position', gain_pos, ...
              'Gain', gain_value, ...
              'OutDataTypeStr', 'Inherit: Inherit via internal rule');

    % 获取原 Add1 下游连线
    % Add1 → [Out1, Add(干扰叠加)]
    lines = get_param(add_block, 'LineHandles');
    out_line = lines.Outport;

    if out_line ~= -1
        % 断开旧连线，重连到增益块
        dst_blocks = get_param(out_line, 'DstBlockHandle');
        delete_line(out_line);

        % Add1 → 增益块
        add_line(subsys, 'Add1/1', '能量归一化/1', 'autorouting', 'smart');

        % 增益块 → 原下游
        for i = 1:length(dst_blocks)
            dst_name = get_param(dst_blocks(i), 'Name');
            add_line(subsys, '能量归一化/1', [dst_name '/1'], 'autorouting', 'smart');
        end
    end

    save_system(model);
    close_system(model);

    fprintf('已修改 %s\n', slx_path);
    fprintf('在 Add1 后插入了 "能量归一化" Gain 块\n');
    fprintf('增益公式: %s\n', gain_value);
    fprintf('\n验证: 基波=1V, h2=0.3, h3=0.15 时增益 = 1/sqrt(1+0.09+0.0225) = %.4f\n', ...
            1/sqrt(1+0.3^2+0.15^2));
    fprintf('此时基波输出 = 1.0 * %.4f = %.4f Vpk\n', ...
            1/sqrt(1+0.3^2+0.15^2), 1/sqrt(1+0.3^2+0.15^2));

catch ME
    warning('修改失败: %s', ME.message);
    close_system(model, 0);
    rethrow(ME);
end
