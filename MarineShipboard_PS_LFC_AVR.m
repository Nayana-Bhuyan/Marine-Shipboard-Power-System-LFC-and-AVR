%% Shipboard Power System: LFC & AVR 
% Author:Nayana Bhuyan
clc; clear; close all;

%% User Inputs
disp('--- Shipboard Power System Simulation ---');
Fref = input('Enter nominal frequency (Hz) [default 50]: ');
if isempty(Fref), Fref = 50; end

Vref = input('Enter nominal voltage (p.u.) [default 1.0]: ');
if isempty(Vref), Vref = 1.0; end

load_change_time = input('Enter time of load disturbance (s) [default 5]: ');
if isempty(load_change_time), load_change_time = 5; end

new_load = input('Enter new load after disturbance (p.u.) [default 1.0]: ');
if isempty(new_load), new_load = 1.0; end

simTime = input('Enter simulation time (s) [default 30]: ');
if isempty(simTime), simTime = 30; end

disp('--- PID Controller Gains ---');
Kp_LFC = input('Enter LFC Kp [default 1.5]: '); if isempty(Kp_LFC), Kp_LFC = 1.5; end
Ki_LFC = input('Enter LFC Ki [default 0.5]: '); if isempty(Ki_LFC), Ki_LFC = 0.5; end
Kd_LFC = input('Enter LFC Kd [default 0.05]: '); if isempty(Kd_LFC), Kd_LFC = 0.05; end

Kp_AVR = input('Enter AVR Kp [default 300]: '); if isempty(Kp_AVR), Kp_AVR = 300; end
Ki_AVR = input('Enter AVR Ki [default 30]: '); if isempty(Ki_AVR), Ki_AVR = 30; end
Kd_AVR = input('Enter AVR Kd [default 10]: '); if isempty(Kd_AVR), Kd_AVR = 10; end

%% System Constants
H = 5; D = 0.015; Tp = 20;
Pe = 0.8;

dt = 0.01;
time = 0:dt:simTime;

%% Initialize Variables
freq = zeros(size(time));
voltage = ones(size(time));
error_f = 0; error_v = 0;
int_error_f = 0; int_error_v = 0;
prev_error_f = 0; prev_error_v = 0;
Pm = 0; Excitation = 0;

%% Simulation Loop
for k = 2:length(time)
    if time(k) >= load_change_time
        Pe = new_load;
    end
    
    % Frequency control
    error_f = Fref - freq(k-1);
    int_error_f = int_error_f + error_f * dt;
    der_error_f = (error_f - prev_error_f) / dt;
    dPm = Kp_LFC * error_f + Ki_LFC * int_error_f + Kd_LFC * der_error_f;
    Pm = Pm + dPm * dt;
    df_dt = (Pm - Pe - D*(freq(k-1)-Fref)) / (2*H);
    freq(k) = freq(k-1) + df_dt * dt;
    prev_error_f = error_f;
    
    % Voltage control
    error_v = Vref - voltage(k-1);
    int_error_v = int_error_v + error_v * dt;
    der_error_v = (error_v - prev_error_v) / dt;
    Excitation = Kp_AVR*error_v + Ki_AVR*int_error_v + Kd_AVR*der_error_v;
    dv_dt = (Excitation - voltage(k-1)) / Tp;
    voltage(k) = voltage(k-1) + dv_dt * dt;
    prev_error_v = error_v;
end

%% Performance Metrics
freq_error = abs(freq - Fref);
voltage_error = abs(voltage - Vref);
settling_freq_idx = find(freq_error < 0.01, 1);
settling_voltage_idx = find(voltage_error < 0.01, 1);
freq_overshoot = max(freq) - Fref;
voltage_overshoot = max(voltage) - Vref;

fprintf('\n--- Performance Metrics ---\n');
fprintf('Frequency Settling Time: %.2f s\n', time(settling_freq_idx));
fprintf('Voltage Settling Time: %.2f s\n', time(settling_voltage_idx));
fprintf('Frequency Overshoot: %.4f Hz\n', freq_overshoot);
fprintf('Voltage Overshoot: %.4f p.u.\n', voltage_overshoot);

%% Dual Y-Axis Plot
if ~exist('images','dir')
    mkdir('images');
end

figure;
yyaxis left;
freqPlot = plot(time, freq, 'b', 'LineWidth', 2);
ylabel('Frequency (Hz)');
yline(Fref, '--k', 'Nominal Frequency');

yyaxis right;
voltPlot = plot(time, voltage, 'r--', 'LineWidth', 2);
ylabel('Voltage (p.u.)');

xlabel('Time (s)');
title('LFC & AVR Response (Dual Y-Axis)');
legend([freqPlot, voltPlot], {'Frequency (Hz)', 'Voltage (p.u.)'});
grid on;

saveas(gcf, 'images/lfc_avr_dual_axis.png');
