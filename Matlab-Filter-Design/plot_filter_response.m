%% plot_filter_response.m
% Menampilkan respons frekuensi filter HAV dan WBV
% HAV: Wh, fs = 3200 Hz
% WBV: Wd dan Wk, fs = 400 Hz

clear; clc; close all;

%% =========================================================
% Load koefisien hasil export
% =========================================================
load('filter_coefficients_all.mat');

% File .mat harus berisi:
% fs_hav, fs_wbv, sos_wh, sos_wd, sos_wk

fprintf('HAV fs = %.0f Hz\n', fs_hav);
fprintf('WBV fs = %.0f Hz\n', fs_wbv);

%% =========================================================
% Plot respons filter HAV Wh
% =========================================================
plot_response_from_sos(sos_wh, fs_hav, ...
    'Frequency Response Filter HAV Wh', ...
    [1 1600], [-80 20]);

%% =========================================================
% Plot respons filter WBV Wd
% =========================================================
plot_response_from_sos(sos_wd, fs_wbv, ...
    'Frequency Response Filter WBV Wd', ...
    [0.1 200], [-80 20]);

%% =========================================================
% Plot respons filter WBV Wk
% =========================================================
plot_response_from_sos(sos_wk, fs_wbv, ...
    'Frequency Response Filter WBV Wk', ...
    [0.1 200], [-80 20]);

%% =========================================================
% Plot gabungan Wd dan Wk untuk WBV
% =========================================================
[H_wd, F_wd] = get_response_from_sos(sos_wd, fs_wbv);
[H_wk, F_wk] = get_response_from_sos(sos_wk, fs_wbv);

figure;
semilogx(F_wd, 20*log10(abs(H_wd)), 'LineWidth', 1.2);
hold on;
semilogx(F_wk, 20*log10(abs(H_wk)), 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Perbandingan Frequency Response WBV Wd dan Wk');
legend('Wd - X/Y axis', 'Wk - Z axis', 'Location', 'best');
grid on;
xlim([0.1 200]);
ylim([-80 20]);

%% =========================================================
% Plot gabungan semua filter
% =========================================================
[H_wh, F_wh] = get_response_from_sos(sos_wh, fs_hav);

figure;
semilogx(F_wh, 20*log10(abs(H_wh)), 'LineWidth', 1.2);
hold on;
semilogx(F_wd, 20*log10(abs(H_wd)), 'LineWidth', 1.2);
semilogx(F_wk, 20*log10(abs(H_wk)), 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Frequency Response HAV Wh dan WBV Wd/Wk');
legend('Wh - HAV', 'Wd - WBV X/Y', 'Wk - WBV Z', 'Location', 'best');
grid on;
xlim([0.1 1600]);
ylim([-80 20]);

%% =========================================================
% Local function: hitung respons dari SOS coefficient
% Format sos:
% [b0 b1 b2 a1 a2]
% denominator = [1 a1 a2]
% =========================================================
function [H_total, F] = get_response_from_sos(sos, fs)

    nfft = 8192;
    H_total = ones(nfft, 1);

    for i = 1:size(sos, 1)
        b = sos(i, 1:3);
        a = [1 sos(i, 4:5)];

        [H, F] = freqz(b, a, nfft, fs);
        H_total = H_total .* H;
    end
end

%% =========================================================
% Local function: plot respons dari SOS coefficient
% =========================================================
function plot_response_from_sos(sos, fs, plot_title, x_limit, y_limit)

    [H, F] = get_response_from_sos(sos, fs);

    figure;
    semilogx(F, 20*log10(abs(H)), 'LineWidth', 1.2);
    xlabel('Frequency (Hz)');
    ylabel('Magnitude (dB)');
    title(plot_title);
    grid on;
    xlim(x_limit);
    ylim(y_limit);
end