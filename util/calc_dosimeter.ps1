<#
.SYNOPSIS
    Native PowerShell Post-Processing & Analysis Utility for Vibration Dosimeter.
    Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)

.EXAMPLE
    .\util\calc_dosimeter.ps1 -CsvPath "f:\dosimeter.csv"
#>

param (
    [string]$CsvPath = "f:\dosimeter.csv"
)

if (-not (Test-Path $CsvPath)) {
    if (Test-Path "dosimeter.csv") {
        $CsvPath = "dosimeter.csv"
    } else {
        Write-Error "File CSV tidak ditemukan di $CsvPath atau ./dosimeter.csv"
        exit 1
    }
}

$data = Import-Csv -Path $CsvPath
$N = $data.Count
if ($N -eq 0) {
    Write-Error "File CSV kosong atau tidak memiliki entri data valid."
    exit 1
}

$T_8H = 28800.0
$T_total = [double]$N

# HAV Sums
$sum_sq_ahwx = 0.0; $sum_sq_ahwy = 0.0; $sum_sq_ahwz = 0.0; $sum_sq_ahv = 0.0

# WBV Sums
$sum_sq_awx = 0.0; $sum_sq_awy = 0.0; $sum_sq_awz = 0.0; $sum_sq_av = 0.0
$sum_4th_awx = 0.0; $sum_4th_awy = 0.0; $sum_4th_awz = 0.0

foreach ($row in $data) {
    $ahwx = [double]$row.ahwx; $ahwy = [double]$row.ahwy; $ahwz = [double]$row.ahwz; $ahv = [double]$row.ahv
    $awx  = [double]$row.awx;  $awy  = [double]$row.awy;  $awz  = [double]$row.awz;  $av  = [double]$row.av

    $sum_sq_ahwx += [Math]::Pow($ahwx, 2); $sum_sq_ahwy += [Math]::Pow($ahwy, 2); $sum_sq_ahwz += [Math]::Pow($ahwz, 2); $sum_sq_ahv += [Math]::Pow($ahv, 2)
    $sum_sq_awx  += [Math]::Pow($awx, 2);  $sum_sq_awy  += [Math]::Pow($awy, 2);  $sum_sq_awz  += [Math]::Pow($awz, 2);  $sum_sq_av  += [Math]::Pow($av, 2)

    $sum_4th_awx += [Math]::Pow($awx, 4);  $sum_4th_awy += [Math]::Pow($awy, 4);  $sum_4th_awz += [Math]::Pow($awz, 4)
}

# HAV Math
$ahwx_eq = [Math]::Sqrt($sum_sq_ahwx / $N); $ahwy_eq = [Math]::Sqrt($sum_sq_ahwy / $N); $ahwz_eq = [Math]::Sqrt($sum_sq_ahwz / $N)
$ahv_eq  = [Math]::Sqrt($sum_sq_ahv / $N)
$A8_hav  = $ahv_eq * [Math]::Sqrt($T_total / $T_8H)

# WBV Math (kx=1.4, ky=1.4, kz=1.0)
$awx_eq = [Math]::Sqrt($sum_sq_awx / $N); $awy_eq = [Math]::Sqrt($sum_sq_awy / $N); $awz_eq = [Math]::Sqrt($sum_sq_awz / $N)
$av_eq  = [Math]::Sqrt($sum_sq_av / $N)

$awx_w = 1.4 * $awx_eq; $awy_w = 1.4 * $awy_eq; $awz_w = 1.0 * $awz_eq
$A8_wbv_x = $awx_w * [Math]::Sqrt($T_total / $T_8H); $A8_wbv_y = $awy_w * [Math]::Sqrt($T_total / $T_8H); $A8_wbv_z = $awz_w * [Math]::Sqrt($T_total / $T_8H)

$A8_wbv_max = [Math]::Max($A8_wbv_x, [Math]::Max($A8_wbv_y, $A8_wbv_z))
$dom_axis = if ($A8_wbv_max -eq $A8_wbv_x) { "X" } elseif ($A8_wbv_max -eq $A8_wbv_y) { "Y" } else { "Z" }

$vdv_x = 1.4 * [Math]::Pow($sum_4th_awx, 0.25)
$vdv_y = 1.4 * [Math]::Pow($sum_4th_awy, 0.25)
$vdv_z = 1.0 * [Math]::Pow($sum_4th_awz, 0.25)
$vdv_total = [Math]::Pow([Math]::Pow($vdv_x, 4) + [Math]::Pow($vdv_y, 4) + [Math]::Pow($vdv_z, 4), 0.25)

Write-Host "===============================================================================" -ForegroundColor Cyan
Write-Host "         VIBRATION DOSIMETER ANALYSIS REPORT (Native PowerShell 7 Engine)" -ForegroundColor Yellow
Write-Host "               Standards: ISO 5349-1 (HAV) | ISO 2631-1 (WBV)" -ForegroundColor Cyan
Write-Host "===============================================================================" -ForegroundColor Cyan
Write-Host "Input Log File   : $CsvPath"
Write-Host "Total Logged Time: ${N}s ($N samples @ 1 Hz)"
Write-Host "-------------------------------------------------------------------------------"

Write-Host "`n1. HAND-ARM VIBRATION (HAV) ANALYSIS (ISO 5349-1)" -ForegroundColor Green
Write-Host ("  * ahv_eq  (Vector Total) = {0:F4} m/s²" -f $ahv_eq)
Write-Host ("  * A_HAV(8) Vector Total  = {0:F4} m/s²" -f $A8_hav)

Write-Host "`n2. WHOLE-BODY VIBRATION (WBV) ANALYSIS (ISO 2631-1)" -ForegroundColor Green
Write-Host ("  * Dominant Axis A_WBV(8) = {0:F4} m/s² (Axis {1})" -f $A8_wbv_max, $dom_axis)
Write-Host ("  * VDV Total Combined    = {0:F4} m/s^1.75" -f $vdv_total)
Write-Host "===============================================================================" -ForegroundColor Cyan
