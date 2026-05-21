clear;
clc;
close all;

SEND_IP   = '192.168.1.150';
SEND_PORT = 1024;
RECV_PORT = 1024;

SYS_ID  = uint8(0);
COMP_ID = uint8(76);
ACT_ID  = uint8(0);
MODE    = uint8(0);

MSG_ID_CTRL = uint32(59328);
MSG_ID_DATA = uint32(59080);

CRC_EXTRA_CTRL = uint8(226);   

f_start  = 0.5;
f_step   = 0.5;
step_dur = 2;
f_max    = 20.0;
amp      = 5.0;
offset   = 30.0;
dt_send  = 0.01;              

n_steps   = round((f_max - f_start) / f_step) + 1;
total_dur = n_steps * step_dur;
fprintf('Ukupno trajanje: %.0f s  (%d koraka frekvencije)\n', total_dur, n_steps);

txSock = udpport("byte");
rxSock = udpport("byte", "LocalPort", RECV_PORT);

seq = uint8(0);

cap     = ceil(total_dur / dt_send * 1.5);
t_send  = nan(cap, 1);
sp_log  = nan(cap, 1);
t_recv  = nan(cap, 1);
pos_log = nan(cap, 1);
idx_s = 1;
idx_r = 1;

recv_buf = uint8([]);

fprintf('Saljem ARM komandu (MAV_CMD_HORNET_ARM_DISARM, param1=1)...\n');
arm_frame = arm_hornet(SYS_ID, COMP_ID, seq);
write(txSock, arm_frame, "uint8", SEND_IP, SEND_PORT);
seq = uint8(mod(double(seq) + 1, 256));
pause(0.5);
fprintf('ARM poslat. Krece chirp...\n');

fprintf('start\n');
t0 = tic;
next_send_time = 0;
last_step_idx = -1;

while true
    elapsed = toc(t0);
    if elapsed < next_send_time
        pause(next_send_time - elapsed);
    end

    t = toc(t0);
    if t >= total_dur, break; end

    step_idx = floor(t / step_dur);
    f_now    = f_start + step_idx * f_step;
    pos_sp   = single(offset + amp * sin(2*pi * f_now * t));

    payload = encode_ctrl_hornet(ACT_ID, MODE, pos_sp);
    frame   = mavlink_v2_frame(SYS_ID, COMP_ID, MSG_ID_CTRL, payload, ...
                seq, CRC_EXTRA_CTRL);

    write(txSock, frame, "uint8", SEND_IP, SEND_PORT);
    seq = uint8(mod(double(seq) + 1, 256));

    t_send(idx_s) = t;
    sp_log(idx_s) = pos_sp;
    idx_s = idx_s + 1;

    if rxSock.NumBytesAvailable > 0
        fresh    = read(rxSock, rxSock.NumBytesAvailable, "uint8");
        recv_buf = [recv_buf, fresh];
        [pos_act, recv_buf] = decode_power_hornet_actid(recv_buf, MSG_ID_DATA, uint8(0));
        if ~isnan(pos_act)
            t_recv(idx_r)  = t;
            pos_log(idx_r) = pos_act;
            idx_r = idx_r + 1;
        end
    end

    if step_idx ~= last_step_idx
        fprintf('  t=%5.1fs  f=%.2f Hz  sp=%.2f mm\n', t, f_now, pos_sp);
        last_step_idx = step_idx;
    end

    next_send_time = next_send_time + dt_send;
end

delete(txSock);
delete(rxSock);
clear txSock rxSock;
fprintf('Test gotov. Primljeno %d odgovora.\n', idx_r-1);

t_send  = t_send(1:idx_s-1);
sp_log  = sp_log(1:idx_s-1);
t_recv  = t_recv(1:idx_r-1);
pos_log = pos_log(1:idx_r-1);

figure('Name','Hornet Chirp Test','NumberTitle','off','Position',[100 100 1100 700]);

ax1 = subplot(2,1,1);
plot(t_send, sp_log,  'b-', 'LineWidth',1.2, 'DisplayName','Setpoint [slato]'); hold on;
plot(t_recv, pos_log, 'r-', 'LineWidth',1.0, 'DisplayName','pos\_act [primljeno]');
xlabel('Vreme (s)');  ylabel('Pozicija (mm)');
title('Chirp: Setpoint vs Izmerena Pozicija');
legend('Location','northwest');  grid on;

freqs = f_start : f_step : f_max;
for k = 1:n_steps
    xline(ax1, (k-1)*step_dur, '--k', 'Alpha',0.3);
    text(ax1, (k-1)*step_dur + 0.1, offset+amp*1.05, ...
         sprintf('%.1fHz', f_start+(k-1)*f_step), 'FontSize',7, 'Color',[.4 .4 .4]);
end

amp_ratio = nan(size(freqs));

for k = 1:length(freqs)
    t0k = (k-1)*step_dur;
    t1k =  k   *step_dur;

    ms = t_send >= t0k & t_send < t1k;
    mr = t_recv >= t0k & t_recv < t1k;

    if sum(ms) < 5 || sum(mr) < 5, continue; end

    A_in  = amp;
    A_out = (max(pos_log(mr)) - min(pos_log(mr))) / 2;
    amp_ratio(k) = 20*log10(max(A_out, 1e-6) / A_in);
end

subplot(2,1,2);
plot(freqs, amp_ratio, 'bo-', 'LineWidth',1.5, 'MarkerFaceColor','b');
yline(-3, 'r--', '-3 dB', 'LabelHorizontalAlignment','right');
ylabel('Amplituda (dB)');  ylim([-30 5]);
xlabel('Frekvencija (Hz)');
title('Frekvencijski Odgovor – Propusni Opseg');
grid on;

valid = ~isnan(amp_ratio);
if sum(valid) > 1
    f_bw = interp1(amp_ratio(valid), freqs(valid), -3, 'linear', 'extrap');
    xline(f_bw, 'r-', sprintf('BW ≈ %.2f Hz', f_bw), 'LabelHorizontalAlignment','left');
    fprintf('Procenjeni propusni opseg (-3dB): %.2f Hz\n', f_bw);
end

function payload = encode_ctrl_hornet(act_id, mode, pos_sp)
    payload = [typecast(single(pos_sp), 'uint8'), uint8(act_id), uint8(mode)];
    last = find(payload ~= 0, 1, 'last');
    if isempty(last)
        payload = uint8(0);
    else
        payload = payload(1:last);
    end
end

function [pos_act, buf_out] = decode_power_hornet_actid(buf, target_msg_id, target_act_id)
    pos_act = NaN;
    buf_out = buf;
    idx = 1;
    last_complete = 0;

    while idx <= length(buf) - 11
        if buf(idx) ~= 253
            idx = idx + 1;
            continue;
        end

        plen = double(buf(idx+1));
        frame_len = 10 + plen + 2;

        if idx + frame_len - 1 > length(buf)
            break;
        end

        mid = uint32(buf(idx+7)) + ...
              bitshift(uint32(buf(idx+8)),  8) + ...
              bitshift(uint32(buf(idx+9)), 16);

        if mid == target_msg_id && plen >= 5
            payload = buf(idx+10 : idx+9+plen);
            act_id = payload(5);
            if act_id == target_act_id
                pos_act = typecast(uint8(payload(1:4)), 'single');
            end
        end

        last_complete = idx + frame_len - 1;
        idx = idx + frame_len;
    end

    if last_complete > 0
        buf_out = buf(last_complete+1 : end);
    else
        buf_out = buf;
    end
end

function frame = mavlink_v2_frame(sys_id, comp_id, msg_id, payload, seq, crc_extra)
    payload = uint8(payload);
    plen    = uint8(length(payload));
    mid_b   = typecast(uint32(msg_id), 'uint8');
    header  = [uint8(253), plen, uint8(0), uint8(0), ...
               uint8(seq), uint8(sys_id), uint8(comp_id), ...
               mid_b(1), mid_b(2), mid_b(3)];
    crc_data = [header(2:end), payload, uint8(crc_extra)];
    crc_val  = x25_crc(crc_data);
    frame    = [header, payload, typecast(uint16(crc_val), 'uint8')];
end

function crc = x25_crc(data)
    crc = uint16(hex2dec('FFFF'));
    for i = 1:numel(data)
        b   = uint8(data(i));
        tmp = uint8(bitxor(b, uint8(bitand(crc, uint16(255)))));
        tmp = uint8(bitxor(tmp, uint8(bitand(bitshift(uint16(tmp), 4), uint16(255)))));
        crc = bitxor(bitshift(crc, -8), ...
              bitxor(bitshift(uint16(tmp), 8), ...
              bitxor(bitshift(uint16(tmp), 3), ...
                     uint16(bitshift(uint8(tmp), -4)))));
    end
end

function frame = arm_hornet(sys_id, comp_id, seq)
    MSG_ID_COMMAND_LONG = uint32(76);
    CRC_EXTRA_CMD_LONG  = uint8(152);

    param1   = single(1);
    params   = zeros(1, 6, 'single');
    command  = uint16(59329);
    tgt_sys  = uint8(1);
    tgt_comp = uint8(1);
    confirm  = uint8(0);

    payload = [typecast(param1,   'uint8'), ...
               typecast(params,   'uint8'), ...
               typecast(command,  'uint8'), ...
               tgt_sys, tgt_comp, confirm];

    frame = mavlink_v2_frame(sys_id, comp_id, MSG_ID_COMMAND_LONG, ...
                             payload, seq, CRC_EXTRA_CMD_LONG);
end