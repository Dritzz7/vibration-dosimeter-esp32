% Export koefisien filter untuk sistem TA

% HAV:
%   fs = 3200 Hz
%   X, Y, Z menggunakan Wh

% WBV:
%   fs = 200 Hz
%   X, Y menggunakan Wd
%   Z menggunakan Wk

% Format tiap section:
% [b0, b1, b2, a1, a2]

clear; clc;
format long g;

fs_hav = 3200;   % Hz
fs_wbv = 400;    % Hz

fprintf('Export koefisien filter\n');
fprintf('HAV fs = %.0f Hz\n', fs_hav);
fprintf('WBV fs = %.0f Hz\n\n', fs_wbv);

% Generate coefficients
sos_wh = get_wh_coefficients(fs_hav);
sos_wd = get_wd_coefficients(fs_wbv);
sos_wk = get_wk_coefficients(fs_wbv);

disp('=== HAV Wh coefficients [b0 b1 b2 a1 a2] ===');
disp(sos_wh);

disp('=== WBV Wd coefficients [b0 b1 b2 a1 a2] ===');
disp(sos_wd);

disp('=== WBV Wk coefficients [b0 b1 b2 a1 a2] ===');
disp(sos_wk);

% Export HAV header
filename_hav = 'hav_coefficients.h';
fid = fopen(filename_hav, 'w');

fprintf(fid, '// hav_coefficients.h\n');
fprintf(fid, '// Auto-generated from MATLAB\n');
fprintf(fid, '// Sampling rate: %.0f Hz\n', fs_hav);
fprintf(fid, '// Format per section: {b0, b1, b2, a1, a2}\n\n');

fprintf(fid, '#ifndef HAV_COEFFICIENTS_H\n');
fprintf(fid, '#define HAV_COEFFICIENTS_H\n\n');

fprintf(fid, 'const float FS_HAV = %.1ff;\n\n', fs_hav);

fprintf(fid, 'const int NUM_SECTIONS_WH = %d;\n', size(sos_wh, 1));
fprintf(fid, 'const float coeff_wh[NUM_SECTIONS_WH][5] = {\n');

for i = 1:size(sos_wh, 1)
    fprintf(fid, '  {%.12ef, %.12ef, %.12ef, %.12ef, %.12ef}', ...
        sos_wh(i,1), sos_wh(i,2), sos_wh(i,3), sos_wh(i,4), sos_wh(i,5));
    if i < size(sos_wh, 1)
        fprintf(fid, ',');
    end
    fprintf(fid, '\n');
end

fprintf(fid, '};\n\n');
fprintf(fid, '#endif\n');

fclose(fid);

% Export WBV header
filename_wbv = 'wbv_coefficients.h';
fid = fopen(filename_wbv, 'w');

fprintf(fid, '// wbv_coefficients.h\n');
fprintf(fid, '// Auto-generated from MATLAB\n');
fprintf(fid, '// Sampling rate: %.0f Hz\n', fs_wbv);
fprintf(fid, '// Format per section: {b0, b1, b2, a1, a2}\n\n');

fprintf(fid, '#ifndef WBV_COEFFICIENTS_H\n');
fprintf(fid, '#define WBV_COEFFICIENTS_H\n\n');

fprintf(fid, 'const float FS_WBV = %.1ff;\n\n', fs_wbv);

fprintf(fid, 'const int NUM_SECTIONS_WD = %d;\n', size(sos_wd, 1));
fprintf(fid, 'const float coeff_wd[NUM_SECTIONS_WD][5] = {\n');

for i = 1:size(sos_wd, 1)
    fprintf(fid, '  {%.12ef, %.12ef, %.12ef, %.12ef, %.12ef}', ...
        sos_wd(i,1), sos_wd(i,2), sos_wd(i,3), sos_wd(i,4), sos_wd(i,5));
    if i < size(sos_wd, 1)
        fprintf(fid, ',');
    end
    fprintf(fid, '\n');
end

fprintf(fid, '};\n\n');

fprintf(fid, 'const int NUM_SECTIONS_WK = %d;\n', size(sos_wk, 1));
fprintf(fid, 'const float coeff_wk[NUM_SECTIONS_WK][5] = {\n');

for i = 1:size(sos_wk, 1)
    fprintf(fid, '  {%.12ef, %.12ef, %.12ef, %.12ef, %.12ef}', ...
        sos_wk(i,1), sos_wk(i,2), sos_wk(i,3), sos_wk(i,4), sos_wk(i,5));
    if i < size(sos_wk, 1)
        fprintf(fid, ',');
    end
    fprintf(fid, '\n');
end

fprintf(fid, '};\n\n');
fprintf(fid, '#endif\n');

fclose(fid);

% Save MATLAB data
save('filter_coefficients_all.mat', ...
     'fs_hav', 'fs_wbv', 'sos_wh', 'sos_wd', 'sos_wk');

fprintf('\nFile berhasil dibuat:\n');
fprintf('- %s\n', filename_hav);
fprintf('- %s\n', filename_wbv);
fprintf('- filter_coefficients_all.mat\n');

% Local function: normalize section
function row = normalize_section(b, a)
    b = b(:).';
    a = a(:).';

    b = b / a(1);
    a = a / a(1);

    if length(b) < 3
        b = [b zeros(1, 3-length(b))];
    end

    if length(a) < 3
        a = [a zeros(1, 3-length(a))];
    end

    % Final format:
    % [b0 b1 b2 a1 a2]
    row = [b(1), b(2), b(3), a(2), a(3)];
end

% Local function: HAV Wh
function sos_wh = get_wh_coefficients(fs)

    % Parameter ISO 5349 / ISO 8041 Wh
    f1 = 6.310;
    f2 = 1258.9;
    f3 = 15.915;
    f4 = 15.915;

    Q1 = 0.71;
    Q2 = 0.71;
    Q4 = 0.64;

    K = 1;

    w1 = 2*pi*f1;
    w2 = 2*pi*f2;
    w3 = 2*pi*f3;
    w4 = 2*pi*f4;

    % 1) High-pass band-limiting
    bh = [1 0 0];
    ah = [1 w1/Q1 w1^2];
    [bzh, azh] = bilinear(bh, ah, fs);

    % 2) Low-pass band-limiting
    bl = [0 0 w2^2];
    al = [1 w2/Q2 w2^2];
    [bzl, azl] = bilinear(bl, al, fs);

    % 3) Frequency weighting Wh
    bt = K * [1/w3 1];
    at = [1/(w4^2) 1/(Q4*w4) 1];
    [bzt, azt] = bilinear(bt, at, fs);

    sos_wh = [
        normalize_section(bzh, azh);
        normalize_section(bzl, azl);
        normalize_section(bzt, azt)
    ];
end

% Local function: WBV Wd
function sos_wd = get_wd_coefficients(fs)

    % Parameter ISO 8041 Wd
    f1 = 0.4;
    f2 = 100;
    f3 = 2;
    f4 = 2;

    Q1 = 1/sqrt(2);
    Q2 = 1/sqrt(2);
    Q4 = 0.63;

    K = 1;

    w1 = 2*pi*f1;
    w2 = 2*pi*f2;
    w3 = 2*pi*f3;
    w4 = 2*pi*f4;

    % 1) High-pass band-limiting
    bh = [1 0 0];
    ah = [1 w1/Q1 w1^2];
    [bzh, azh] = bilinear(bh, ah, fs);

    % 2) Low-pass band-limiting
    bl = [0 0 w2^2];
    al = [1 w2/Q2 w2^2];
    [bzl, azl] = bilinear(bl, al, fs);

    % 3) A-V transition Wd
    bt = K * [1/w3 1];
    at = [1/(w4^2) 1/(Q4*w4) 1];
    [bzt, azt] = bilinear(bt, at, fs);

    sos_wd = [
        normalize_section(bzh, azh);
        normalize_section(bzl, azl);
        normalize_section(bzt, azt)
    ];
end

% Local function: WBV Wk
function sos_wk = get_wk_coefficients(fs)

    % Parameter ISO 8041 Wk
    f1 = 0.4;
    f2 = 100;
    f3 = 12.5;
    f4 = 12.5;
    f5 = 2.37;
    f6 = 3.35;

    Q1 = 1/sqrt(2);
    Q2 = 1/sqrt(2);
    Q4 = 0.63;
    Q5 = 0.91;
    Q6 = 0.91;

    K = 1;

    w1 = 2*pi*f1;
    w2 = 2*pi*f2;
    w3 = 2*pi*f3;
    w4 = 2*pi*f4;
    w5 = 2*pi*f5;
    w6 = 2*pi*f6;

    % 1) High-pass band-limiting
    bh = [1 0 0];
    ah = [1 w1/Q1 w1^2];
    [bzh, azh] = bilinear(bh, ah, fs);

    % 2) Low-pass band-limiting
    bl = [0 0 w2^2];
    al = [1 w2/Q2 w2^2];
    [bzl, azl] = bilinear(bl, al, fs);

    % 3) A-V transition Wk
    bt = K * [1/w3 1];
    at = [1/(w4^2) 1/(Q4*w4) 1];
    [bzt, azt] = bilinear(bt, at, fs);

    % 4) Upward-step Wk
    bs = (w5/w6)^2 * [1/(w5^2) 1/(Q5*w5) 1];
    as = [1/(w6^2) 1/(Q6*w6) 1];
    [bzs, azs] = bilinear(bs, as, fs);

    sos_wk = [
        normalize_section(bzh, azh);
        normalize_section(bzl, azl);
        normalize_section(bzt, azt);
        normalize_section(bzs, azs)
    ];
end