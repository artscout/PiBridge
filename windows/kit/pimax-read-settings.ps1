# SPDX-License-Identifier: MIT
# Adapted from omarchyvr windows/pimax-read-settings.ps1 (MIT OR Apache-2.0).
<#
  Reads (only reads) Pimax Play settings the way Pimax Play itself does: Runtime\PiPlayService.dll,
  PiService_Get{Int,Float}Config and PiService_GetApp{Int,Float}Config("global", ...). No setter is bound.
  Usage: powershell -ExecutionPolicy Bypass -File .\pimax-read-settings.ps1 [-OutFile <path>]
#>
param([string]$OutFile = '')

$ErrorActionPreference = 'Stop'
$install = (Get-ItemProperty 'HKLM:\SOFTWARE\Pimax' -ErrorAction SilentlyContinue).InstallPath
if (-not $install) { $install = 'C:\Program Files\Pimax' }
$dll = Join-Path $install 'Runtime\PiPlayService.dll'
if (-not (Test-Path $dll)) { throw "PiPlayService.dll not found: $dll" }

$cs = @"
using System;
using System.Runtime.InteropServices;
public static class PiRead {
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)] public static extern IntPtr LoadLibraryW(string p);
    [DllImport("kernel32.dll", CharSet = CharSet.Ansi)] public static extern IntPtr GetProcAddress(IntPtr h, string n);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] public delegate int D_Init();
    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)] public delegate int D_GetI(string key, out int v);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)] public delegate int D_GetF(string key, out float v);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)] public delegate int D_GetAI(string app, string key, out int v);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)] public delegate int D_GetAF(string app, string key, out float v);
    static IntPtr h;
    static T F<T>(string n) where T : class {
        IntPtr p = GetProcAddress(h, n);
        if (p == IntPtr.Zero) throw new Exception("missing export " + n);
        return Marshal.GetDelegateForFunctionPointer(p, typeof(T)) as T;
    }
    public static D_GetI GetI; public static D_GetF GetF; public static D_GetAI GetAI; public static D_GetAF GetAF;
    public static int Open(string dll) {
        h = LoadLibraryW(dll);
        if (h == IntPtr.Zero) throw new Exception("LoadLibrary failed " + Marshal.GetLastWin32Error());
        GetI = F<D_GetI>("PiService_GetIntConfig"); GetF = F<D_GetF>("PiService_GetFloatConfig");
        GetAI = F<D_GetAI>("PiService_GetAppIntConfig"); GetAF = F<D_GetAF>("PiService_GetAppFloatConfig");
        return F<D_Init>("PiServiceInit")();
    }
}
"@
Add-Type -TypeDefinition $cs

$keys = @(
  'runtime_fov_level', 'piplay_fov_level_count', 'piplay_fov_custom_fine_type', 'runtime_fov_fix_degree_offset',
  'piplay_steamvr_use_native_fov', 'runtime_enable_parallel_projection', 'piplay_fov_view_cropping_enable',
  'runtime_dbg_hidden_area_enable',
  'runtime_fov_custom_left_outer_percent', 'runtime_fov_custom_left_inner_percent',
  'runtime_fov_custom_left_upper_percent', 'runtime_fov_custom_left_lower_percent',
  'runtime_fov_custom_right_outer_percent', 'runtime_fov_custom_right_inner_percent',
  'runtime_fov_custom_right_upper_percent', 'runtime_fov_custom_right_lower_percent',
  'piplay_fov_custom_quick_horizontal_percent', 'piplay_fov_custom_quick_vertical_percent',
  'piplay_fov_custom_fine_top_to_center', 'piplay_fov_custom_fine_bottom_to_center',
  'piplay_fov_custom_fine_left_left_to_center', 'piplay_fov_custom_fine_left_right_to_center',
  'piplay_fov_custom_fine_right_left_to_center', 'piplay_fov_custom_fine_right_right_to_center',
  'piplay_display_quality_level', 'runtime_pixels_per_display_pixel_rate', 'runtime_gpu_speed_render_target_scale',
  'runtime_foveated_rendering_level', 'runtime_enable_foveated_rendering', 'runtime_quadviews_rendering_level',
  'runtime_enable_openxr_quadviews_fr', 'runtime_ipd', 'runtime_lens_vertical_offset_0', 'runtime_lens_vertical_offset_1',
  'runtime_lens_horizontal_offset_0', 'runtime_lens_horizontal_offset_1',
  'runtime_physical_ipd', 'piplay_ipd_range_min', 'piplay_ipd_range_max', 'runtime_ipd_auto_adjust',
  'piplay_enable_eye_tracking', 'runtime_refresh_rate', 'piplay_refresh_rate', 'runtime_brightness', 'hmd_volume'
)

$lines = New-Object System.Collections.Generic.List[string]
$init = [PiRead]::Open($dll)
$lines.Add("PiPlayService: $dll  PiServiceInit=$init  time=$(Get-Date -Format s)")
$lines.Add("key`tint`tfloat`tglobalAppInt`tglobalAppFloat   (- = call did not return 0)")
foreach ($k in $keys) {
    $i = 0; $f = 0.0; $ai = 0; $af = 0.0
    $ri = [PiRead]::GetI.Invoke($k, [ref]$i); $rf = [PiRead]::GetF.Invoke($k, [ref]$f)
    $rai = [PiRead]::GetAI.Invoke('global', $k, [ref]$ai); $raf = [PiRead]::GetAF.Invoke('global', $k, [ref]$af)
    $c = @($(if ($ri -eq 0) { "$i" } else { '-' }), $(if ($rf -eq 0) { "$f" } else { '-' }),
           $(if ($rai -eq 0) { "$ai" } else { '-' }), $(if ($raf -eq 0) { "$af" } else { '-' }))
    $lines.Add("$k`t" + ($c -join "`t"))
}
$lines | ForEach-Object { Write-Host $_ }
if ($OutFile -ne '') { $lines | Set-Content -Encoding UTF8 $OutFile }
