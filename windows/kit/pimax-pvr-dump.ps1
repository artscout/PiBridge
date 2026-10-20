# SPDX-License-Identifier: MIT
# Adapted from omarchyvr windows/pimax-dump-distortion.ps1 (MIT OR Apache-2.0). Used by pibridge-headset-data.ps1 (parts 1 and 3 only).
<#
.SYNOPSIS
  Pimax "collect everything" dump for Pimax headsets (written for the Dream Air, works through the generic PVR interface). READ-ONLY towards the headset (query calls only).

.DESCRIPTION
  Loads Pimax's own PVR client (libPVRClient64.dll, shipped with Pimax Play), opens a PVR session and
  queries it. No set/write/recenter/haptic function is ever bound, so nothing can be changed on the headset.

  Parts (menu, or -Parts "1,3"):
    0  environment snapshot (always)            OS, GPU, Pimax processes/services, USB devices of the headset
    1  distortion + FOV + eye poses + hidden-area mesh + render sizes   -> pimax-distortion-<serial>.json
    3  other read-only info (IPD, properties, VST camera calibration, config keys)  -> pimax-info.json
    4  copy Pimax data files + file inventory (names/versions/SHA-256 of dll/exe/sys)
    2  pose logger (head + both controllers + input + gaze), N seconds, CSV  -> pimax-poses.csv
       (run last; start USBPcap first if you want the USB traffic for the same time span)

  Everything goes to .\pimax-windows-dump-<date>\ next to this script, log in pimax-dump.log.

  Written for Windows PowerShell 5.1 (inline C# 5, no compiler needed).
#>
[CmdletBinding()]
param(
    [string]$Parts = '',          # e.g. "1,3,4,2"; empty = ask (Enter = everything)
    [int]$Seconds = 120,          # length of the pose logger
    [int]$GridN = 129,            # distortion grid is GridN x GridN per eye and channel
    [int]$LineN = 1024,           # samples of each radial line
    [string]$DllPath = '',        # override the location of libPVRClient64.dll
    [string]$OutRoot = '',        # default: folder of this script
    [switch]$NoPause
)

$ErrorActionPreference = 'Continue'
$ToolVersion = '1.1'
if ($OutRoot -eq '') { $OutRoot = $PSScriptRoot }
if ($OutRoot -eq '' -or $OutRoot -eq $null) { $OutRoot = (Get-Location).Path }
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$OutDir = Join-Path $OutRoot ("pimax-windows-dump-" + $stamp)
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$LogPath = Join-Path $OutDir 'pimax-dump.log'

$cs = @'
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Diagnostics;

namespace PimaxDump
{
    // ---------------------------------------------------------------- logging
    public static class Log
    {
        static StreamWriter w;
        static readonly object lk = new object();
        public static void Init(string path)
        {
            w = new StreamWriter(path, true, new UTF8Encoding(false));
            w.AutoFlush = true;
        }
        static void Put(string tag, string s, ConsoleColor c)
        {
            string line = DateTime.Now.ToString("HH:mm:ss.fff") + " " + tag + " " + s;
            lock (lk)
            {
                if (w != null) w.WriteLine(line);
                try
                {
                    ConsoleColor old = Console.ForegroundColor;
                    Console.ForegroundColor = c;
                    Console.WriteLine(line);
                    Console.ForegroundColor = old;
                }
                catch (Exception) { }
            }
        }
        public static void Info(string s) { Put("[info]", s, ConsoleColor.Gray); }
        public static void Ok(string s) { Put("[ ok ]", s, ConsoleColor.Green); }
        public static void Warn(string s) { Put("[warn]", s, ConsoleColor.Yellow); }
        public static void Err(string s) { Put("[FAIL]", s, ConsoleColor.Red); }
        public static bool Ready { get { return w != null; } }
        public static void FileOnly(string line) { lock (lk) { if (w != null) w.WriteLine(line); } }
    }

    // ---------------------------------------------------------------- tiny JSON writer
    public class Jw
    {
        StringBuilder sb = new StringBuilder();
        List<int> counts = new List<int>();
        bool pendingKey = false;
        static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
        void Pre()
        {
            if (pendingKey) { pendingKey = false; return; }
            if (counts.Count > 0)
            {
                if (counts[counts.Count - 1] > 0) sb.Append(',');
                counts[counts.Count - 1]++;
            }
        }
        public static string Q(string s)
        {
            if (s == null) return "null";
            StringBuilder b = new StringBuilder("\"");
            foreach (char c in s)
            {
                switch (c)
                {
                    case '"': b.Append("\\\""); break;
                    case '\\': b.Append("\\\\"); break;
                    case '\n': b.Append("\\n"); break;
                    case '\r': b.Append("\\r"); break;
                    case '\t': b.Append("\\t"); break;
                    default:
                        if (c < 32) b.Append("\\u" + ((int)c).ToString("x4")); else b.Append(c);
                        break;
                }
            }
            return b.Append('"').ToString();
        }
        public static string N(double d)
        {
            if (double.IsNaN(d) || double.IsInfinity(d)) return "null";
            return d.ToString("R", Inv);
        }
        public static string N(float f)
        {
            if (float.IsNaN(f) || float.IsInfinity(f)) return "null";
            return f.ToString("R", Inv);
        }
        public Jw Obj() { Pre(); sb.Append('{'); counts.Add(0); return this; }
        public Jw EndObj() { counts.RemoveAt(counts.Count - 1); sb.Append('}'); return this; }
        public Jw Arr() { Pre(); sb.Append('['); counts.Add(0); return this; }
        public Jw EndArr() { counts.RemoveAt(counts.Count - 1); sb.Append(']'); return this; }
        public Jw Key(string k) { Pre(); sb.Append(Q(k)).Append(':'); pendingKey = true; return this; }
        public Jw S(string v) { Pre(); sb.Append(Q(v)); return this; }
        public Jw D(double v) { Pre(); sb.Append(N(v)); return this; }
        public Jw F(float v) { Pre(); sb.Append(N(v)); return this; }
        public Jw I(long v) { Pre(); sb.Append(v.ToString(Inv)); return this; }
        public Jw B(bool v) { Pre(); sb.Append(v ? "true" : "false"); return this; }
        public Jw Null() { Pre(); sb.Append("null"); return this; }
        public Jw KS(string k, string v) { return Key(k).S(v); }
        public Jw KD(string k, double v) { return Key(k).D(v); }
        public Jw KF(string k, float v) { return Key(k).F(v); }
        public Jw KI(string k, long v) { return Key(k).I(v); }
        public Jw KB(string k, bool v) { return Key(k).B(v); }
        public Jw FloatArr(float[] a, int n)
        {
            Pre(); sb.Append('[');
            for (int i = 0; i < n; i++) { if (i > 0) sb.Append(','); sb.Append(N(a[i])); }
            sb.Append(']'); return this;
        }
        public Jw FloatArr(float[] a) { return FloatArr(a, a.Length); }
        public override string ToString() { return sb.ToString(); }
        public void Save(string path) { File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false)); }
    }

    // ---------------------------------------------------------------- native structs (layout from PVR_Types.h)
    [StructLayout(LayoutKind.Sequential)] public struct Vec2 { public float x, y; }
    [StructLayout(LayoutKind.Sequential)] public struct Vec3 { public float x, y, z; }
    [StructLayout(LayoutKind.Sequential)] public struct Quat { public float x, y, z, w; }
    [StructLayout(LayoutKind.Sequential)] public struct Posef { public Quat q; public Vec3 p; }
    [StructLayout(LayoutKind.Sequential)] public struct FovPort { public float Up, Down, Left, Right; }
    [StructLayout(LayoutKind.Sequential)] public struct Recti { public int x, y, w, h; }

    [StructLayout(LayoutKind.Sequential)]
    public struct EyeRenderInfo { public FovPort Fov; public Vec2 PixelsPerTanAngleAtCenter; public Posef HmdToEyePose; public Recti DistortedViewport; }

    [StructLayout(LayoutKind.Sequential)]
    public struct DisplayInfo
    {
        public int edid_vid, edid_pid, disp_state, eye_rotate, eye_display, width, height, pos_x, pos_y;
        public float refresh_rate;
        public long luid;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HmdInfoN
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)] public byte[] ProductName;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)] public byte[] Manufacturer;
        public int VendorId, ProductId;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 24)] public byte[] SerialNumber;
        public int FirmwareMajor, FirmwareMinor;
        public int ResW, ResH;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HmdStatusN { public byte IsVisible, HmdPresent, HmdMounted, DisplayLost, ServiceReady, ShouldQuit, HasInputFocus; }

    [StructLayout(LayoutKind.Sequential)]
    public struct PoseState
    {
        public Posef ThePose;
        public Vec3 AngularVelocity, LinearVelocity, AngularAcceleration, LinearAcceleration;
        public double TimeInSeconds;
        public uint StatusFlags;
        public uint pad;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TrackingState { public PoseState Head, Hand0, Hand1; }

    [StructLayout(LayoutKind.Sequential)]
    public struct EyeTrackingInfo
    {
        public Vec2 GazeL, GazeR;
        public double TimeInSeconds;
        public float ConvergenceDistance;
        public float BlinkL, BlinkR;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct InputState
    {
        public double TimeInSeconds;
        public uint Buttons0, Buttons1, Touches0, Touches1;
        public float Trigger0, Trigger1, Grip0, Grip1;
        public Vec2 TouchPad0, TouchPad1, Joy0, Joy1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TrackerDesc { public float HFov, VFov, NearZ, FarZ; }

    // ---------------------------------------------------------------- delegates (only query functions exist here)
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_Init();
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate void D_Void();
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_Create(out IntPtr h);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate void D_Destroy(IntPtr h);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate IntPtr D_Str();
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate double D_Time();
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_H_P(IntPtr h, IntPtr p);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_H_E_P(IntPtr h, int eye, IntPtr p);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_H_T_P(IntPtr h, double t, IntPtr p);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_DevPose(IntPtr h, int dev, double t, IntPtr p);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_GetOrigin(IntPtr h, out int o);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_Caps(IntPtr h, int dev, out uint cap);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate float D_CfgF(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, float def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_CfgI(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, int def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate long D_CfgL(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, long def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_CfgS(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string key, IntPtr buf, int size);
    // pvrFovPort is 16 bytes: the Win64 ABI passes it by pointer to a caller-side copy -> IntPtr
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_FovTex(IntPtr h, int eye, IntPtr fov, float ppdp, IntPtr size);
    // pvrVector2f is 8 bytes: the Win64 ABI passes it in an integer register -> ulong (low 32 bits = x, high = y)
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_DistUV(IntPtr h, int eye, ulong uv, IntPtr out3);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate uint D_Count(IntPtr h);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_H_U_P(IntPtr h, uint idx, IntPtr p);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_Devices(IntPtr h, out uint devs);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate float D_PropF(IntPtr h, int dev, int prop, float def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_PropI(IntPtr h, int dev, int prop, int def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate long D_PropL(IntPtr h, int dev, int prop, long def);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_PropS(IntPtr h, int dev, int prop, IntPtr buf, int size);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate uint D_Hidden(IntPtr h, int eye, int type, IntPtr buf, uint n);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_H_Ret(IntPtr h);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_VstDist(IntPtr h, uint cam, out int type, IntPtr k8);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_VstIntr(IntPtr h, uint cam, out uint w, out uint hh, IntPtr focal, IntPtr center);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int D_VstExtr(IntPtr h, uint cam, IntPtr pose);

    // ---------------------------------------------------------------- the tool
    public static class Tool
    {
        // slot numbers in pvrInterfaceV32 (PVR_Interface.h), checked by script against the header
        const int S_INIT = 0, S_SHUTDOWN = 1, S_CREATE = 2, S_DESTROY = 3, S_VERSTR = 4, S_TIME = 5, S_HMDINFO = 6,
            S_DISPINFO = 7, S_EYERENDER = 8, S_HMDSTATUS = 9, S_GETORIGIN = 11, S_TRACKSTATE = 13, S_DEVCAPS = 14,
            S_INPUT = 15, S_GETFLOAT = 16, S_GETINT = 18, S_GETSTR = 20, S_FOVTEX = 23, S_DISTUV = 24,
            S_TRACKERCOUNT = 44, S_TRACKERDESC = 45, S_TRACKERPOSE = 46, S_CONNDEV = 52, S_DEVPOSE = 53,
            S_PROPF = 54, S_PROPI = 55, S_PROPS = 56, S_GETI64 = 60, S_PROPI64 = 64, S_EYETRACK = 65,
            S_HIDDEN2 = 68, S_VSTTYPE = 69, S_VSTFMT = 70, S_VSTDIST = 71, S_VSTINTR = 72, S_VSTEXTR = 73;

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        static extern IntPtr LoadLibraryExW(string name, IntPtr file, uint flags);
        [DllImport("kernel32.dll", CharSet = CharSet.Ansi, SetLastError = true)]
        static extern IntPtr GetProcAddress(IntPtr h, string name);
        [DllImport("kernel32.dll")] static extern void GetSystemTimePreciseAsFileTime(out long ft);
        [DllImport("winmm.dll")] static extern uint timeBeginPeriod(uint ms);
        [DllImport("winmm.dll")] static extern uint timeEndPeriod(uint ms);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate IntPtr D_GetIface(uint major, uint minor);

        static IntPtr dll = IntPtr.Zero, iface = IntPtr.Zero, hmd = IntPtr.Zero;
        static bool inited = false;
        public static string DllFullPath = "", InterfaceVersion = "";
        static IntPtr buf = IntPtr.Zero;                 // scratch (zeroed before each use)
        const int BUFSZ = 1 << 16;
        static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
        static Dictionary<int, Delegate> cache = new Dictionary<int, Delegate>();

        static T Fn<T>(int slot) where T : class
        {
            Delegate d;
            if (cache.TryGetValue(slot, out d)) return (T)(object)d;
            IntPtr p = Marshal.ReadIntPtr(iface, slot * IntPtr.Size);
            if (p == IntPtr.Zero) return null;
            d = Marshal.GetDelegateForFunctionPointer(p, typeof(T));
            cache[slot] = d;
            return (T)(object)d;
        }
        static void Clear() { for (int i = 0; i < 2048; i += 8) Marshal.WriteInt64(buf, i, 0); }
        static string Cstr(byte[] b)
        {
            if (b == null) return "";
            int n = 0; while (n < b.Length && b[n] != 0) n++;
            return Encoding.UTF8.GetString(b, 0, n);
        }
        static string CstrPtr(IntPtr p, int max)
        {
            if (p == IntPtr.Zero) return "";
            byte[] b = new byte[max]; Marshal.Copy(p, b, 0, max); return Cstr(b);
        }
        static T Rd<T>() { return (T)Marshal.PtrToStructure(buf, typeof(T)); }
        static T RdAt<T>(int off) { return (T)Marshal.PtrToStructure(new IntPtr(buf.ToInt64() + off), typeof(T)); }
        public static string QpcNow() { return Stopwatch.GetTimestamp().ToString(Inv); }

        public static string ResultName(int r)
        {
            string[] n = { "success", "failed", "dll_failed", "dll_wrong", "interface_not_found", "invalid_param", "rpc_failed",
                "share_mem_failed", "unsupport_render_name", "no_display", "no_render_device", "app_not_visible", "srv_not_ready",
                "dll_srv_mismatch", "app_adapter_mismatch", "not_support" };
            if (r >= 0 && r < n.Length) return n[r] + "(" + r + ")";
            return "code(" + r + ")";
        }

        // ------------------------------------------------------------ connect
        public static bool Connect(string dllPath, int waitSeconds)
        {
            if (buf == IntPtr.Zero) buf = Marshal.AllocHGlobal(BUFSZ);
            if (!Environment.Is64BitProcess) { Log.Err("This is a 32-bit PowerShell; start the 64-bit one (the .cmd launcher does)."); return false; }
            DllFullPath = dllPath;
            uint LOAD_WITH_ALTERED_SEARCH_PATH = 0x8;
            dll = LoadLibraryExW(dllPath, IntPtr.Zero, LOAD_WITH_ALTERED_SEARCH_PATH);
            if (dll == IntPtr.Zero)
            {
                Log.Err("LoadLibrary failed for " + dllPath + " (win32 error " + Marshal.GetLastWin32Error() + ")");
                return false;
            }
            IntPtr pf = GetProcAddress(dll, "getPvrInterface");
            if (pf == IntPtr.Zero) { Log.Err("getPvrInterface is not exported by " + dllPath + ". Not a PVR client DLL?"); return false; }
            D_GetIface gi = (D_GetIface)Marshal.GetDelegateForFunctionPointer(pf, typeof(D_GetIface));
            // PVR_MAJOR_VERSION 1, PVR_MINOR_VERSION 32 in the open PVR headers; try older minors if the runtime refuses
            uint[] minors = { 32, 31, 30, 29, 28 };
            foreach (uint mi in minors)
            {
                iface = gi(1, mi);
                if (iface != IntPtr.Zero) { InterfaceVersion = "1." + mi; break; }
                Log.Warn("getPvrInterface(1," + mi + ") returned NULL");
            }
            if (iface == IntPtr.Zero) { Log.Err("The runtime does not provide any PVR interface 1.28-1.32."); return false; }
            if (InterfaceVersion != "1.32") Log.Warn("Interface version " + InterfaceVersion + ": slot layout may differ from the 1.32 header; results may be wrong.");
            Log.Ok("PVR interface " + InterfaceVersion + " from " + dllPath);

            int r = Fn<D_Init>(S_INIT)();
            if (r != 0) { Log.Err("pvr_initialise failed: " + ResultName(r) + ". Is Pimax Play / PiService running?"); return false; }
            inited = true;
            IntPtr vs = Fn<D_Str>(S_VERSTR)();
            Log.Info("PVR version string: " + CstrPtr(vs, 64));

            DateTime end = DateTime.UtcNow.AddSeconds(waitSeconds);
            while (true)
            {
                r = Fn<D_Create>(S_CREATE)(out hmd);
                if (r == 0 && hmd != IntPtr.Zero) break;
                Log.Warn("pvr_createSession: " + ResultName(r));
                if (DateTime.UtcNow > end) { Log.Err("No PVR session. Start Pimax Play, connect the headset, put it on, retry."); return false; }
                Thread.Sleep(2000);
            }
            Log.Ok("PVR session created");
            while (true)
            {
                Clear();
                r = Fn<D_H_P>(S_HMDSTATUS)(hmd, buf);
                HmdStatusN st = Rd<HmdStatusN>();
                Log.Info("hmd status: result=" + ResultName(r) + " ServiceReady=" + st.ServiceReady + " HmdPresent=" + st.HmdPresent
                    + " Mounted=" + st.HmdMounted + " ShouldQuit=" + st.ShouldQuit);
                if (r == 0 && st.ServiceReady != 0 && st.HmdPresent != 0) break;
                if (DateTime.UtcNow > end) { Log.Warn("Headset not reported present/ready; continuing anyway (data may be empty)."); break; }
                Thread.Sleep(2000);
            }
            return true;
        }

        public static void Close()
        {
            try
            {
                if (hmd != IntPtr.Zero) { Fn<D_Destroy>(S_DESTROY)(hmd); hmd = IntPtr.Zero; }
                if (inited) { Fn<D_Void>(S_SHUTDOWN)(); inited = false; }
            }
            catch (Exception e) { Log.Warn("close: " + e.Message); }
        }

        public static string Serial()
        {
            Clear();
            if (Fn<D_H_P>(S_HMDINFO)(hmd, buf) != 0) return "";
            return Cstr(Rd<HmdInfoN>().SerialNumber);
        }

        // ------------------------------------------------------------ small readers
        static void WriteHmdInfo(Jw j)
        {
            Clear();
            int r = Fn<D_H_P>(S_HMDINFO)(hmd, buf);
            HmdInfoN h = Rd<HmdInfoN>();
            j.Obj().KS("result", ResultName(r));
            j.KS("product_name", Cstr(h.ProductName)).KS("manufacturer", Cstr(h.Manufacturer));
            j.KI("vendor_id", h.VendorId).KI("product_id", h.ProductId).KS("serial", Cstr(h.SerialNumber));
            j.KI("firmware_major", h.FirmwareMajor).KI("firmware_minor", h.FirmwareMinor);
            j.Key("resolution").Arr().I(h.ResW).I(h.ResH).EndArr().EndObj();
        }
        static void WriteStatus(Jw j)
        {
            Clear();
            int r = Fn<D_H_P>(S_HMDSTATUS)(hmd, buf);
            HmdStatusN s = Rd<HmdStatusN>();
            j.Obj().KS("result", ResultName(r)).KB("is_visible", s.IsVisible != 0).KB("hmd_present", s.HmdPresent != 0)
                .KB("hmd_mounted", s.HmdMounted != 0).KB("display_lost", s.DisplayLost != 0).KB("service_ready", s.ServiceReady != 0)
                .KB("should_quit", s.ShouldQuit != 0).KB("has_input_focus", s.HasInputFocus != 0).EndObj();
        }
        static void WriteDisplay(Jw j, int eye)
        {
            Clear();
            int r = Fn<D_H_E_P>(S_DISPINFO)(hmd, eye, buf);
            DisplayInfo d = Rd<DisplayInfo>();
            string[] states = { "none", "direct", "extend", "virtual" };
            string[] disp = { "horz_side_by_side", "vert_side_by_side", "full" };
            j.Obj().KS("result", ResultName(r)).KI("edid_vid", d.edid_vid).KI("edid_pid", d.edid_pid)
                .KS("disp_state", (d.disp_state >= 0 && d.disp_state < 4) ? states[d.disp_state] : d.disp_state.ToString())
                .KI("eye_rotate_deg", d.eye_rotate * 90).KS("eye_display", (d.eye_display >= 0 && d.eye_display < 3) ? disp[d.eye_display] : d.eye_display.ToString())
                .KI("width", d.width).KI("height", d.height).KI("pos_x", d.pos_x).KI("pos_y", d.pos_y)
                .KF("refresh_rate", d.refresh_rate).EndObj();
        }
        static void WritePosef(Jw j, Posef p)
        {
            j.Obj().Key("position").Arr().F(p.p.x).F(p.p.y).F(p.p.z).EndArr()
                .Key("orientation_xyzw").Arr().F(p.q.x).F(p.q.y).F(p.q.z).F(p.q.w).EndArr().EndObj();
        }
        static float[] QuatToEulerDeg(Quat q)
        {
            // yaw (about +y), pitch (about +x), roll (about +z) for a right-handed y-up frame; for information only
            double sinp = 2 * (q.w * q.x - q.y * q.z);
            double pitch = Math.Asin(Math.Max(-1, Math.Min(1, sinp)));
            double yaw = Math.Atan2(2 * (q.w * q.y + q.x * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
            double roll = Math.Atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.x * q.x + q.z * q.z));
            return new float[] { (float)(yaw * 180 / Math.PI), (float)(pitch * 180 / Math.PI), (float)(roll * 180 / Math.PI) };
        }

        static string CfgString(string key)
        {
            Clear();
            int n = Fn<D_CfgS>(S_GETSTR)(hmd, key, buf, 512);
            if (n <= 0) return null;
            return CstrPtr(buf, 512);
        }

        // ------------------------------------------------------------ PART 1: distortion
        public static string DumpDistortion(string outDir, int gridN, int lineN, string toolVersion)
        {
            string serial = Serial();
            string name = "pimax-distortion-" + (serial != "" ? serial : DateTime.Now.ToString("yyyyMMdd-HHmmss")) + ".json";
            string path = Path.Combine(outDir, name);
            Jw j = new Jw();
            j.Obj();
            j.KS("tool", "pimax-dump-distortion").KS("tool_version", toolVersion).KI("format", 1);
            j.KS("timestamp_utc", DateTime.UtcNow.ToString("o")).KS("machine", Environment.MachineName);
            j.Key("dll").Obj().KS("path", DllFullPath).KS("interface_version", InterfaceVersion);
            try
            {
                FileVersionInfo fv = FileVersionInfo.GetVersionInfo(DllFullPath);
                j.KS("file_version", fv.FileVersion).KS("product_version", fv.ProductVersion);
            }
            catch (Exception) { }
            j.EndObj();
            j.KS("pvr_version_string", CstrPtr(Fn<D_Str>(S_VERSTR)(), 64));
            j.Key("hmd"); WriteHmdInfo(j);
            j.Key("status"); WriteStatus(j);
            j.Key("display").Obj(); j.Key("left"); WriteDisplay(j, 0); j.Key("right"); WriteDisplay(j, 1); j.EndObj();
            // settings that change what getEyeRenderInfo / the distortion mean (READ only)
            j.Key("config").Obj();
            j.KI("view_rotation_fix_default_-1", Fn<D_CfgI>(S_GETINT)(hmd, "view_rotation_fix", -1));
            j.KI("no_render_default_-1", Fn<D_CfgI>(S_GETINT)(hmd, "no_render", -1));
            j.KF("ipd_mm_default_-1", Fn<D_CfgF>(S_GETFLOAT)(hmd, "ipd", -1f));
            j.KS("hmd_codename", CfgString("hmd_codename"));
            j.EndObj();

            string[] eyeName = { "left", "right" };
            j.Key("direction").S("pvr_getHmdDistortedUV: input = panel UV of the eye (0..1 over the eye's native panel, before display rotation), "
                + "output = per channel UV (0..1) into the undistorted render texture that spans the eye's Fov port; see README/format notes");
            j.Key("eyes").Arr();
            for (int eye = 0; eye < 2; eye++)
            {
                Log.Info("eye " + eyeName[eye] + ": render info");
                j.Obj().KS("eye", eyeName[eye]);
                Clear();
                int r = Fn<D_H_E_P>(S_EYERENDER)(hmd, eye, buf);
                EyeRenderInfo ri = Rd<EyeRenderInfo>();
                j.KS("render_info_result", ResultName(r));
                j.Key("fov_tan").Obj().KF("up", ri.Fov.Up).KF("down", ri.Fov.Down).KF("left", ri.Fov.Left).KF("right", ri.Fov.Right).EndObj();
                float rad = (float)(180 / Math.PI);
                j.Key("fov_deg").Obj().KF("up", (float)(Math.Atan(ri.Fov.Up) * rad)).KF("down", (float)(Math.Atan(ri.Fov.Down) * rad))
                    .KF("left", (float)(Math.Atan(ri.Fov.Left) * rad)).KF("right", (float)(Math.Atan(ri.Fov.Right) * rad)).EndObj();
                j.Key("pixels_per_tan_angle_at_center").Arr().F(ri.PixelsPerTanAngleAtCenter.x).F(ri.PixelsPerTanAngleAtCenter.y).EndArr();
                j.Key("hmd_to_eye_pose"); WritePosef(j, ri.HmdToEyePose);
                float[] eu = QuatToEulerDeg(ri.HmdToEyePose.q);
                j.Key("hmd_to_eye_euler_deg_yaw_pitch_roll").Arr().F(eu[0]).F(eu[1]).F(eu[2]).EndArr();
                j.Key("distorted_viewport").Obj().KI("x", ri.DistortedViewport.x).KI("y", ri.DistortedViewport.y)
                    .KI("w", ri.DistortedViewport.w).KI("h", ri.DistortedViewport.h).EndObj();

                // recommended texture size at 1.0 pixel per display pixel
                IntPtr fovp = Marshal.AllocHGlobal(16);
                Marshal.StructureToPtr(ri.Fov, fovp, false);
                Clear();
                r = Fn<D_FovTex>(S_FOVTEX)(hmd, eye, fovp, 1.0f, buf);
                Marshal.FreeHGlobal(fovp);
                j.Key("recommended_texture_size").Obj().KS("result", ResultName(r)).KI("w", Marshal.ReadInt32(buf, 0)).KI("h", Marshal.ReadInt32(buf, 4)).EndObj();

                // texture sizes for small and one-sided FOV ports (read-only query): the runtime's own pixels per tan
                // at the centre and per half, in x and y, to pin down the panel pixel scale PVR assumes per axis
                {
                    float[][] ports = {
                        new float[] { 0.05f, 0.05f, 0.05f, 0.05f }, new float[] { 0.2f, 0.2f, 0.2f, 0.2f },
                        new float[] { 0.5f, 0.001f, 0.001f, 0.001f }, new float[] { 0.001f, 0.5f, 0.001f, 0.001f },
                        new float[] { 0.001f, 0.001f, 0.5f, 0.001f }, new float[] { 0.001f, 0.001f, 0.001f, 0.5f } };
                    j.Key("fov_texture_size_probe").Arr();
                    foreach (float[] fp in ports)
                    {
                        FovPort pf = new FovPort { Up = fp[0], Down = fp[1], Left = fp[2], Right = fp[3] };
                        IntPtr pp = Marshal.AllocHGlobal(16);
                        Marshal.StructureToPtr(pf, pp, false);
                        Clear();
                        int rp = Fn<D_FovTex>(S_FOVTEX)(hmd, eye, pp, 1.0f, buf);
                        Marshal.FreeHGlobal(pp);
                        j.Obj().KF("up", fp[0]).KF("down", fp[1]).KF("left", fp[2]).KF("right", fp[3]).KS("result", ResultName(rp))
                            .KI("w", Marshal.ReadInt32(buf, 0)).KI("h", Marshal.ReadInt32(buf, 4)).EndObj();
                    }
                    j.EndArr();
                }

                // hidden area meshes: 0 hidden triangles, 1 visible triangles, 2 border line list, 3 visible rectangle
                string[] mt = { "hidden_area_triangles", "visible_area_triangles", "border_line_list", "visible_rectangle" };
                j.Key("meshes").Obj();
                for (int t = 0; t < 4; t++)
                {
                    j.Key(mt[t]);
                    try
                    {
                        uint n = Fn<D_Hidden>(S_HIDDEN2)(hmd, eye, t, IntPtr.Zero, 0);
                        if (n > 100000) n = 0;
                        IntPtr vb = Marshal.AllocHGlobal((int)(n * 8 + 8));
                        for (uint z = 0; z < n * 8 + 8; z += 8) Marshal.WriteInt64(vb, (int)z, 0);
                        uint got = n > 0 ? Fn<D_Hidden>(S_HIDDEN2)(hmd, eye, t, vb, n) : 0;
                        j.Obj().KI("vertex_count", n).KI("filled", got).Key("vertices_uv").Arr();
                        for (uint v = 0; v < n && v < got; v++)
                        {
                            j.Arr().F(BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(vb, (int)(v * 8))), 0))
                             .F(BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(vb, (int)(v * 8 + 4))), 0)).EndArr();
                        }
                        j.EndArr().EndObj();
                        Marshal.FreeHGlobal(vb);
                        Log.Info("  mesh " + mt[t] + ": " + n + " vertices");
                    }
                    catch (Exception e) { j.Null(); Log.Warn("  mesh " + mt[t] + ": " + e.Message); }
                }
                j.EndObj();

                // distortion samples
                D_DistUV f = Fn<D_DistUV>(S_DISTUV);
                IntPtr o3 = Marshal.AllocHGlobal(64);
                Stopwatch sw = Stopwatch.StartNew();
                int fails = 0; int firstFail = 0;
                j.Key("distortion").Obj();
                j.KS("convention", "panel_uv -> render_uv");
                j.KI("grid_n", gridN);
                j.KS("grid_layout", "row-major, v outer loop (v=j/(n-1)), u inner loop (u=i/(n-1)); arrays hold [u0,v0,u1,v1,...] of the OUTPUT uv");
                float[][] gr = new float[3][];
                for (int c = 0; c < 3; c++) gr[c] = new float[gridN * gridN * 2];
                for (int jj = 0; jj < gridN; jj++)
                {
                    for (int ii = 0; ii < gridN; ii++)
                    {
                        float u = ii / (float)(gridN - 1), v = jj / (float)(gridN - 1);
                        ulong packed = ((ulong)BitConverter.ToUInt32(BitConverter.GetBytes(v), 0) << 32) | BitConverter.ToUInt32(BitConverter.GetBytes(u), 0);
                        for (int z = 0; z < 24; z += 8) Marshal.WriteInt64(o3, z, 0);
                        int rr = f(hmd, eye, packed, o3);
                        if (rr != 0) { fails++; if (firstFail == 0) firstFail = rr; }
                        int idx = (jj * gridN + ii) * 2;
                        for (int c = 0; c < 3; c++)
                        {
                            gr[c][idx] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, c * 8)), 0);
                            gr[c][idx + 1] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, c * 8 + 4)), 0);
                        }
                    }
                }
                j.Key("grid_r").FloatArr(gr[0]).Key("grid_g").FloatArr(gr[1]).Key("grid_b").FloatArr(gr[2]);
                // radial lines from the centre (0.5,0.5) to the edge of the unit square along +x, -x, +y, -y
                j.KI("line_n", lineN).KS("line_layout", "sample k: s=k/(n-1); input uv = (0.5,0.5)+dir*0.5*s; arrays [u0,v0,u1,v1,...] of the OUTPUT uv");
                string[] dn = { "+x", "-x", "+y", "-y" };
                float[] dx = { 1, -1, 0, 0 }, dy = { 0, 0, 1, -1 };
                j.Key("lines").Obj();
                for (int d = 0; d < 4; d++)
                {
                    float[][] ln = new float[3][];
                    for (int c = 0; c < 3; c++) ln[c] = new float[lineN * 2];
                    for (int k = 0; k < lineN; k++)
                    {
                        float s = k / (float)(lineN - 1);
                        float u = 0.5f + dx[d] * 0.5f * s, v = 0.5f + dy[d] * 0.5f * s;
                        ulong packed = ((ulong)BitConverter.ToUInt32(BitConverter.GetBytes(v), 0) << 32) | BitConverter.ToUInt32(BitConverter.GetBytes(u), 0);
                        for (int z = 0; z < 24; z += 8) Marshal.WriteInt64(o3, z, 0);
                        int rr = f(hmd, eye, packed, o3);
                        if (rr != 0) { fails++; if (firstFail == 0) firstFail = rr; }
                        for (int c = 0; c < 3; c++)
                        {
                            ln[c][k * 2] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, c * 8)), 0);
                            ln[c][k * 2 + 1] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, c * 8 + 4)), 0);
                        }
                    }
                    j.Key(dn[d]).Obj().Key("r").FloatArr(ln[0]).Key("g").FloatArr(ln[1]).Key("b").FloatArr(ln[2]).EndObj();
                }
                j.EndObj();
                // extended grid beyond the unit square (green only + the call result per point), to see the domain PVR
                // really uses: whether it clamps at 0/1, continues to a square over the longer panel side, or has a rim.
                // Failures here are kept apart from failed_calls (they are expected outside the domain).
                {
                    int en = 97; float lo = -0.25f, hi = 1.25f;
                    float[] eg = new float[en * en * 2]; int[] er = new int[en * en];
                    for (int jj = 0; jj < en; jj++)
                        for (int ii = 0; ii < en; ii++)
                        {
                            float u = lo + (hi - lo) * ii / (en - 1), v = lo + (hi - lo) * jj / (en - 1);
                            ulong packed = ((ulong)BitConverter.ToUInt32(BitConverter.GetBytes(v), 0) << 32) | BitConverter.ToUInt32(BitConverter.GetBytes(u), 0);
                            for (int z = 0; z < 24; z += 8) Marshal.WriteInt64(o3, z, 0);
                            int rr = f(hmd, eye, packed, o3);
                            int idx = jj * en + ii;
                            er[idx] = rr;
                            eg[idx * 2] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, 8)), 0);
                            eg[idx * 2 + 1] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(o3, 12)), 0);
                        }
                    j.Key("distortion_extended").Obj().KI("grid_n", en).KF("lo", lo).KF("hi", hi)
                        .KS("layout", "input uv = lo+(hi-lo)*i/(n-1), v outer loop; grid_g holds [u,v] of the green OUTPUT uv; result = call result per point")
                        .Key("grid_g").FloatArr(eg).Key("result").Arr();
                    foreach (int x in er) j.I(x);
                    j.EndArr().EndObj();
                }
                j.KI("failed_calls", fails).KS("first_failure", fails > 0 ? ResultName(firstFail) : "none");
                j.EndObj();
                Marshal.FreeHGlobal(o3);
                Log.Ok("eye " + eyeName[eye] + ": " + (gridN * gridN + 4 * lineN) + " distortion points in " + sw.ElapsedMilliseconds + " ms, failed calls: " + fails);
                j.EndObj();
            }
            j.EndArr();
            j.EndObj();
            j.Save(path);
            Log.Ok("wrote " + path);
            return path;
        }

        // ------------------------------------------------------------ PART 3: other info
        static readonly string[] PropNames = {
            "None", "RenderModelTranslation_Vector3f", "RenderModelRotation_Quatf", "BatteryLevel_int", "BatteryPercent_int",
            "PoseRefreshRate_Float", "TrackerHFovInRadians_Float", "TrackerVFovInRadians_Float", "TrackerNearZInMeters_Float",
            "TrackerFarZInMeters_Float", "Product_String", "Manufacturer_String", "VenderId_int", "ProductId_int",
            "RenderModelName_String", "InputProfilePath_String", "ControllerType_String", "Serial_String", "ModeLabel_String",
            "Firmware_UpdateAvailable_Bool", "Firmware_ForceUpdateRequired_Bool", "Firmware_ManualUpdate_Bool",
            "Firmware_ManualUpdateURL_String", "Firmware_ProgrammingTarget_String", "TrackingFirmwareVersion_String",
            "FirmwareVersion_Uint64", "RegisteredDeviceType_String", "HardwareRevision_Uint64", "HardwareRevision_String",
            "ResourceRoot_String", "FPGAVersion_Uint64", "VRCVersion_Uint64", "RadioVersion_Uint64", "DongleVersion_Uint64",
            "Identifiable_Bool", "ConnectedWirelessDongle_String", "InputButtons_Uint64", "Audio_PlaybackDeviceId_String",
            "Audio_RecordingDeviceId_String", "Prop_HmdTrackingStyle_Int", "ScreenState_Bool", "SecondsFromVsyncToPhotons_Float",
            "IpdRangeMin_Float", "IpdRangeMax_Float", "UniverseId_int", "SleepMode_Bool", "SupportsEyeTracking_Bool" };

        static readonly string[] CfgKeys = {
            // known from CustomHeadsetOpenVR
            "ipd", "eye_height", "no_render", "hmd_codename", "view_rotation_fix", "hmd_buttons", "enable_foveated_rendering",
            // guesses (a key the runtime does not know simply comes back as 'unset')
            "brightness", "hmd_brightness", "refresh_rate", "hmd_refresh_rate", "display_refresh_rate", "fov", "fov_level",
            "fov_scale", "render_quality", "pitch_tilt", "smart_smoothing", "eye_tracking", "eye_tracking_enabled",
            "foveated_rendering", "dfr_level", "lens_ipd", "ipd_motor", "ipd_position", "ipd_mm", "hmd_serial", "hmd_sn",
            "hmd_fw_version", "firmware_version", "hmd_model", "hmd_name", "tracking_mode", "slam_enabled", "openport",
            "hmd_brightness_level", "hmd_volume", "wifi", "color_mode", "gamma", "contrast", "saturation",
            "parallel_projection", "compatible_with_openvr", "hidden_area_mask", "vst_enabled", "world_scale", "world_scale_percent"
        };

        public static string DumpInfo(string outDir, string[] extraKeys)
        {
            string path = Path.Combine(outDir, "pimax-info.json");
            Jw j = new Jw();
            j.Obj().KS("tool", "pimax-dump-distortion").KS("part", "info").KS("timestamp_utc", DateTime.UtcNow.ToString("o"));
            j.KS("pvr_version_string", CstrPtr(Fn<D_Str>(S_VERSTR)(), 64)).KS("interface_version", InterfaceVersion);
            j.Key("hmd"); WriteHmdInfo(j);
            j.Key("status"); WriteStatus(j);
            j.Key("display").Obj(); j.Key("left"); WriteDisplay(j, 0); j.Key("right"); WriteDisplay(j, 1); j.EndObj();
            int origin = -1;
            int ro = Fn<D_GetOrigin>(S_GETORIGIN)(hmd, out origin);
            j.Key("tracking_origin").Obj().KS("result", ResultName(ro)).KI("value_0_eye_1_floor", origin).EndObj();

            // eye render info + IPD derived from the two eye poses
            Posef[] pos = new Posef[2];
            for (int e = 0; e < 2; e++) { Clear(); Fn<D_H_E_P>(S_EYERENDER)(hmd, e, buf); pos[e] = Rd<EyeRenderInfo>().HmdToEyePose; }
            double dxm = pos[0].p.x - pos[1].p.x, dym = pos[0].p.y - pos[1].p.y, dzm = pos[0].p.z - pos[1].p.z;
            j.KD("ipd_mm_from_eye_poses", Math.Sqrt(dxm * dxm + dym * dym + dzm * dzm) * 1000.0);

            // connected devices
            uint devs = 0;
            int rc = Fn<D_Devices>(S_CONNDEV)(hmd, out devs);
            j.Key("connected_devices").Obj().KS("result", ResultName(rc)).KI("bitmask", devs).EndObj();
            Log.Info("connected device bitmask: 0x" + devs.ToString("x"));

            // per-device properties (HMD always, controllers / trackers if connected)
            j.Key("device_properties").Obj();
            int[] devIds = { 0x1, 0x2, 0x4, 0x8, 0x10 };
            string[] devNames = { "hmd", "left_controller", "right_controller", "tracker0", "tracker1" };
            for (int di = 0; di < devIds.Length; di++)
            {
                if (di > 0 && (devs & (uint)devIds[di]) == 0) continue;
                j.Key(devNames[di]).Obj();
                Clear();
                uint cap = 0; Fn<D_Caps>(S_DEVCAPS)(hmd, devIds[di], out cap);
                j.KI("caps_bitmask_1_orientation_2_position", cap);
                for (int p = 3; p < PropNames.Length; p++)
                {
                    string pn = PropNames[p];
                    try
                    {
                        if (pn.EndsWith("_Vector3f") || pn.EndsWith("_Quatf")) continue;
                        if (pn.EndsWith("_String"))
                        {
                            Clear();
                            int n = Fn<D_PropS>(S_PROPS)(hmd, devIds[di], p, buf, 512);
                            if (n > 0) j.KS(pn, CstrPtr(buf, 512));
                        }
                        else if (pn.EndsWith("_Float"))
                        {
                            float a = Fn<D_PropF>(S_PROPF)(hmd, devIds[di], p, -12345.0f), b = Fn<D_PropF>(S_PROPF)(hmd, devIds[di], p, 54321.0f);
                            if (a == b) j.KF(pn, a);
                        }
                        else if (pn.EndsWith("_Uint64"))
                        {
                            long a = Fn<D_PropL>(S_PROPI64)(hmd, devIds[di], p, -1L), b = Fn<D_PropL>(S_PROPI64)(hmd, devIds[di], p, -2L);
                            if (a == b) j.KS(pn, "0x" + a.ToString("x") + " (" + a.ToString(Inv) + ")");
                        }
                        else
                        {
                            int a = Fn<D_PropI>(S_PROPI)(hmd, devIds[di], p, -12345), b = Fn<D_PropI>(S_PROPI)(hmd, devIds[di], p, -54321);
                            if (a == b) j.KI(pn, a);
                        }
                    }
                    catch (Exception e) { Log.Warn("prop " + pn + ": " + e.Message); }
                }
                j.EndObj();
            }
            j.EndObj();

            // config keys: a key is 'set' only if two different defaults both come back unchanged-equal
            j.Key("config_keys_read_only_probe").Obj();
            List<string> keys = new List<string>(CfgKeys);
            if (extraKeys != null) foreach (string k in extraKeys) if (k.Length > 0) keys.Add(k);
            foreach (string k in keys)
            {
                try
                {
                    int ia = Fn<D_CfgI>(S_GETINT)(hmd, k, -987654), ib = Fn<D_CfgI>(S_GETINT)(hmd, k, -123456);
                    float fa = Fn<D_CfgF>(S_GETFLOAT)(hmd, k, -9876.5f), fb = Fn<D_CfgF>(S_GETFLOAT)(hmd, k, -1234.5f);
                    long la = Fn<D_CfgL>(S_GETI64)(hmd, k, -987654321L), lb = Fn<D_CfgL>(S_GETI64)(hmd, k, -123456789L);
                    string s = CfgString(k);
                    bool anySet = (ia == ib) || (fa == fb) || (la == lb) || (s != null && s.Length > 0);
                    if (!anySet) continue;
                    j.Key(k).Obj();
                    if (ia == ib) j.KI("int", ia);
                    if (fa == fb) j.KF("float", fa);
                    if (la == lb) j.KI("int64", la);
                    if (s != null && s.Length > 0) j.KS("string", s);
                    j.EndObj();
                    Log.Info("config '" + k + "' is set");
                }
                catch (Exception e) { Log.Warn("config " + k + ": " + e.Message); }
            }
            j.EndObj();

            // trackers (cameras of the inside-out system as seen by PVR)
            j.Key("trackers").Arr();
            try
            {
                uint tc = Fn<D_Count>(S_TRACKERCOUNT)(hmd);
                if (tc > 16) tc = 0;
                for (uint t = 0; t < tc; t++)
                {
                    Clear();
                    int r1 = Fn<D_H_U_P>(S_TRACKERDESC)(hmd, t, buf);
                    TrackerDesc td = Rd<TrackerDesc>();
                    Clear();
                    int r2 = Fn<D_H_U_P>(S_TRACKERPOSE)(hmd, t, buf);
                    uint flags = (uint)Marshal.ReadInt32(buf, 0);
                    Posef tp = RdAt<Posef>(4);
                    j.Obj().KI("index", t).KS("desc_result", ResultName(r1)).KF("hfov_rad", td.HFov).KF("vfov_rad", td.VFov)
                        .KF("near_z", td.NearZ).KF("far_z", td.FarZ).KS("pose_result", ResultName(r2)).KI("status_flags", flags);
                    j.Key("pose"); WritePosef(j, tp); j.EndObj();
                }
            }
            catch (Exception e) { Log.Warn("trackers: " + e.Message); }
            j.EndArr();

            // video see-through cameras: intrinsics / distortion / camera->hmd extrinsics (useful for our SLAM calibration)
            j.Key("vst").Obj();
            try
            {
                int vt = Fn<D_H_Ret>(S_VSTTYPE)(hmd), vf = Fn<D_H_Ret>(S_VSTFMT)(hmd);
                j.KS("type_0none_1mono_2stereo", vt.ToString()).KS("stream_format_0unk_1nv12_2raw8", vf.ToString());
                j.Key("cameras").Arr();
                int ncam = vt == 1 ? 1 : (vt == 2 ? 2 : 0);
                for (uint c = 0; c < (uint)ncam; c++)
                {
                    Clear();
                    uint w = 0, hh = 0;
                    IntPtr foc = new IntPtr(buf.ToInt64() + 256), cen = new IntPtr(buf.ToInt64() + 264);
                    int r1 = Fn<D_VstIntr>(S_VSTINTR)(hmd, c, out w, out hh, foc, cen);
                    Vec2 fl = (Vec2)Marshal.PtrToStructure(foc, typeof(Vec2)), ce = (Vec2)Marshal.PtrToStructure(cen, typeof(Vec2));
                    int dtype = 0;
                    IntPtr k8 = new IntPtr(buf.ToInt64() + 512);
                    int r2 = Fn<D_VstDist>(S_VSTDIST)(hmd, c, out dtype, k8);
                    float[] kk = new float[8];
                    for (int i = 0; i < 8; i++) kk[i] = BitConverter.ToSingle(BitConverter.GetBytes(Marshal.ReadInt32(k8, i * 4)), 0);
                    IntPtr pp = new IntPtr(buf.ToInt64() + 768);
                    int r3 = Fn<D_VstExtr>(S_VSTEXTR)(hmd, c, pp);
                    Posef ex = (Posef)Marshal.PtrToStructure(pp, typeof(Posef));
                    if (r1 != 0 && r2 != 0 && r3 != 0) continue;
                    j.Obj().KI("camera", c).KS("intrinsics_result", ResultName(r1)).KI("width", w).KI("height", hh)
                        .Key("focal").Arr().F(fl.x).F(fl.y).EndArr().Key("center").Arr().F(ce.x).F(ce.y).EndArr()
                        .KS("distortion_result", ResultName(r2)).KI("distortion_type_1_fisheye4", dtype).Key("k").FloatArr(kk)
                        .KS("extrinsics_result", ResultName(r3)).Key("camera_to_hmd"); WritePosef(j, ex); j.EndObj();
                }
                j.EndArr();
            }
            catch (Exception e) { Log.Warn("vst: " + e.Message); }
            j.EndObj();

            // one snapshot of the tracking / input / gaze structs
            j.Key("snapshot").Obj();
            double now = Fn<D_Time>(S_TIME)();
            Clear();
            int rt = Fn<D_H_T_P>(S_TRACKSTATE)(hmd, now, buf);
            TrackingState ts = Rd<TrackingState>();
            j.KS("tracking_state_result", ResultName(rt)).KD("pvr_time", now).KI("head_status_flags", ts.Head.StatusFlags);
            j.Key("head_pose"); WritePosef(j, ts.Head.ThePose);
            Clear();
            int ri2 = Fn<D_H_P>(S_INPUT)(hmd, buf);
            j.KS("input_state_result", ResultName(ri2));
            Clear();
            int rg = Fn<D_H_T_P>(S_EYETRACK)(hmd, now, buf);
            EyeTrackingInfo gi = Rd<EyeTrackingInfo>();
            j.KS("eye_tracking_result", ResultName(rg)).Key("gaze_tan_left").Arr().F(gi.GazeL.x).F(gi.GazeL.y).EndArr()
                .Key("gaze_tan_right").Arr().F(gi.GazeR.x).F(gi.GazeR.y).EndArr().KF("convergence_distance", gi.ConvergenceDistance)
                .KF("blink_left", gi.BlinkL).KF("blink_right", gi.BlinkR);
            j.EndObj();
            j.EndObj();
            j.Save(path);
            Log.Ok("wrote " + path);
            return path;
        }

        // ------------------------------------------------------------ PART 2: pose logger
        static void PoseCols(StringBuilder sb, PoseState s)
        {
            sb.Append(',').Append(Jw.N(s.TimeInSeconds)).Append(',').Append(s.StatusFlags);
            sb.Append(',').Append(Jw.N(s.ThePose.p.x)).Append(',').Append(Jw.N(s.ThePose.p.y)).Append(',').Append(Jw.N(s.ThePose.p.z));
            sb.Append(',').Append(Jw.N(s.ThePose.q.x)).Append(',').Append(Jw.N(s.ThePose.q.y)).Append(',').Append(Jw.N(s.ThePose.q.z)).Append(',').Append(Jw.N(s.ThePose.q.w));
            sb.Append(',').Append(Jw.N(s.AngularVelocity.x)).Append(',').Append(Jw.N(s.AngularVelocity.y)).Append(',').Append(Jw.N(s.AngularVelocity.z));
            sb.Append(',').Append(Jw.N(s.LinearVelocity.x)).Append(',').Append(Jw.N(s.LinearVelocity.y)).Append(',').Append(Jw.N(s.LinearVelocity.z));
            sb.Append(',').Append(Jw.N(s.AngularAcceleration.x)).Append(',').Append(Jw.N(s.AngularAcceleration.y)).Append(',').Append(Jw.N(s.AngularAcceleration.z));
            sb.Append(',').Append(Jw.N(s.LinearAcceleration.x)).Append(',').Append(Jw.N(s.LinearAcceleration.y)).Append(',').Append(Jw.N(s.LinearAcceleration.z));
        }
        static string PoseHdr(string p)
        {
            string[] f = { "time", "status", "px", "py", "pz", "qx", "qy", "qz", "qw", "wx", "wy", "wz", "vx", "vy", "vz", "awx", "awy", "awz", "ax", "ay", "az" };
            StringBuilder sb = new StringBuilder();
            foreach (string s in f) sb.Append(',').Append(p).Append('_').Append(s);
            return sb.ToString();
        }

        // Polls at the highest rate Sleep(1) allows (timer period 1 ms) and writes every poll. t_qpc is the raw
        // QueryPerformanceCounter tick count (Stopwatch.GetTimestamp), utc_filetime = GetSystemTimePreciseAsFileTime
        // (100 ns ticks since 1601-01-01 UTC, the same clock USBPcap/Wireshark stamp packets with).
        public static string LogPoses(string outDir, int seconds)
        {
            string csv = Path.Combine(outDir, "pimax-poses.csv");
            string meta = Path.Combine(outDir, "pimax-poses-meta.json");
            Jw m = new Jw();
            m.Obj().KS("tool", "pimax-dump-distortion").KS("part", "pose-logger").KI("seconds_requested", seconds);
            m.KI("qpc_frequency_hz", Stopwatch.Frequency);
            long ft0; GetSystemTimePreciseAsFileTime(out ft0);
            long q0 = Stopwatch.GetTimestamp(); double pvr0 = Fn<D_Time>(S_TIME)();
            m.KI("start_qpc", q0).KI("start_utc_filetime", ft0).KS("start_utc_iso", DateTime.UtcNow.ToString("o")).KD("start_pvr_time_s", pvr0);
            m.KS("columns_note", "t_qpc = raw QPC ticks; utc_filetime = 100ns ticks since 1601 UTC (precise); pvr_now = pvr_getTimeSeconds; "
                + "<dev>_time = pose TimeInSeconds from PVR; w* = angular velocity rad/s; v* = linear velocity m/s; aw*/a* = accelerations (the only IMU-like data PVR exposes)");

            StringBuilder hdr = new StringBuilder("t_qpc,utc_filetime,pvr_now,track_res" );
            hdr.Append(PoseHdr("head")).Append(PoseHdr("lhand")).Append(PoseHdr("rhand"));
            hdr.Append(",input_res,input_time,btn_l,btn_r,touch_l,touch_r,trig_l,trig_r,grip_l,grip_r,pad_lx,pad_ly,pad_rx,pad_ry,joy_lx,joy_ly,joy_rx,joy_ry");
            hdr.Append(",gaze_res,gaze_time,gazeL_x,gazeL_y,gazeR_x,gazeR_y,gaze_conv,blinkL,blinkR");
            int rows = 0, newHead = 0; double lastHead = -1;
            uint tb = timeBeginPeriod(1);
            D_H_T_P fTrack = Fn<D_H_T_P>(S_TRACKSTATE); D_H_P fIn = Fn<D_H_P>(S_INPUT); D_H_T_P fGaze = Fn<D_H_T_P>(S_EYETRACK); D_Time fTime = Fn<D_Time>(S_TIME);
            // separate buffers so the three reads stay valid together
            IntPtr b1 = Marshal.AllocHGlobal(1024), b2 = Marshal.AllocHGlobal(1024), b3 = Marshal.AllocHGlobal(1024);
            using (StreamWriter w = new StreamWriter(csv, false, new UTF8Encoding(false), 1 << 16))
            {
                w.WriteLine(hdr.ToString());
                long freq = Stopwatch.Frequency, endTicks = q0 + (long)seconds * freq, lastReport = q0, lastFlush = q0;
                StringBuilder sb = new StringBuilder();
                string stopFlag = Path.Combine(outDir, "stop.flag");   // capture-session.ps1 creates it to end the take cleanly
                int errs = 0;
                try {
                while (true)
                {
                    long q = Stopwatch.GetTimestamp();
                    if (q >= endTicks) break;
                    // flush at least once per second (a killed process used to lose the last 1 MiB block, 2-4 s)
                    if (q - lastFlush >= freq) { lastFlush = q; w.Flush(); if (File.Exists(stopFlag)) { Log.Info("pose logger: stop.flag seen, ending"); break; } }
                    try {
                    long ft; GetSystemTimePreciseAsFileTime(out ft);
                    double now = fTime();
                    for (int z = 0; z < 1024; z += 8) { Marshal.WriteInt64(b1, z, 0); Marshal.WriteInt64(b2, z, 0); Marshal.WriteInt64(b3, z, 0); }
                    int rt = fTrack(hmd, now, b1);
                    int ri = fIn(hmd, b2);
                    int rg = fGaze(hmd, now, b3);
                    TrackingState ts = (TrackingState)Marshal.PtrToStructure(b1, typeof(TrackingState));
                    InputState inp = (InputState)Marshal.PtrToStructure(b2, typeof(InputState));
                    EyeTrackingInfo gz = (EyeTrackingInfo)Marshal.PtrToStructure(b3, typeof(EyeTrackingInfo));
                    sb.Length = 0;
                    sb.Append(q).Append(',').Append(ft).Append(',').Append(Jw.N(now)).Append(',').Append(rt);
                    PoseCols(sb, ts.Head); PoseCols(sb, ts.Hand0); PoseCols(sb, ts.Hand1);
                    sb.Append(',').Append(ri).Append(',').Append(Jw.N(inp.TimeInSeconds)).Append(',').Append(inp.Buttons0).Append(',').Append(inp.Buttons1)
                      .Append(',').Append(inp.Touches0).Append(',').Append(inp.Touches1)
                      .Append(',').Append(Jw.N(inp.Trigger0)).Append(',').Append(Jw.N(inp.Trigger1)).Append(',').Append(Jw.N(inp.Grip0)).Append(',').Append(Jw.N(inp.Grip1))
                      .Append(',').Append(Jw.N(inp.TouchPad0.x)).Append(',').Append(Jw.N(inp.TouchPad0.y)).Append(',').Append(Jw.N(inp.TouchPad1.x)).Append(',').Append(Jw.N(inp.TouchPad1.y))
                      .Append(',').Append(Jw.N(inp.Joy0.x)).Append(',').Append(Jw.N(inp.Joy0.y)).Append(',').Append(Jw.N(inp.Joy1.x)).Append(',').Append(Jw.N(inp.Joy1.y));
                    sb.Append(',').Append(rg).Append(',').Append(Jw.N(gz.TimeInSeconds)).Append(',').Append(Jw.N(gz.GazeL.x)).Append(',').Append(Jw.N(gz.GazeL.y))
                      .Append(',').Append(Jw.N(gz.GazeR.x)).Append(',').Append(Jw.N(gz.GazeR.y)).Append(',').Append(Jw.N(gz.ConvergenceDistance))
                      .Append(',').Append(Jw.N(gz.BlinkL)).Append(',').Append(Jw.N(gz.BlinkR));
                    w.WriteLine(sb.ToString());
                    rows++;
                    errs = 0;
                    if (ts.Head.TimeInSeconds != lastHead) { newHead++; lastHead = ts.Head.TimeInSeconds; }
                    if (q - lastReport >= freq * 10)
                    {
                        lastReport = q;
                        Log.Info("pose logger: " + ((q - q0) / freq) + " s, " + rows + " rows, " + newHead + " distinct head poses, last head pos "
                            + Jw.N(ts.Head.ThePose.p.x) + "," + Jw.N(ts.Head.ThePose.p.y) + "," + Jw.N(ts.Head.ThePose.p.z) + " track_res=" + rt);
                    }
                    } catch (Exception ex) {
                        // a game starting/stopping can make PVR calls fail for a moment: keep logging (the game take stopped after 28 s)
                        errs++;
                        if (errs == 1 || errs % 100 == 0) Log.Info("pose logger: read failed (" + errs + " in a row): " + ex.Message);
                        if (errs > 5000) { Log.Info("pose logger: giving up after 5000 failed reads"); break; }
                        Thread.Sleep(20);
                        continue;
                    }
                    Thread.Sleep(1);
                }
                } finally { w.Flush(); }
            }
            if (tb == 0) timeEndPeriod(1);
            Marshal.FreeHGlobal(b1); Marshal.FreeHGlobal(b2); Marshal.FreeHGlobal(b3);
            long ft1; GetSystemTimePreciseAsFileTime(out ft1);
            long q1 = Stopwatch.GetTimestamp();
            m.KI("end_qpc", q1).KI("end_utc_filetime", ft1).KS("end_utc_iso", DateTime.UtcNow.ToString("o")).KD("end_pvr_time_s", Fn<D_Time>(S_TIME)());
            m.KI("rows", rows).KI("distinct_head_poses", newHead).KD("mean_rows_per_s", rows / Math.Max(1e-9, (q1 - q0) / (double)Stopwatch.Frequency));
            m.EndObj(); m.Save(meta);
            Log.Ok("pose log: " + rows + " rows, " + newHead + " distinct head poses -> " + csv);
            return csv;
        }
    }
}
'@

# ------------------------------------------------------------------ helpers (PowerShell side)
function Write-Log([string]$msg) {
    $line = (Get-Date -Format 'HH:mm:ss.fff') + ' [ps  ] ' + $msg
    # once [PimaxDump.Log] has the file open, Add-Content can't share it; write through the same handle
    if (('PimaxDump.Log' -as [type]) -and [PimaxDump.Log]::Ready) { [PimaxDump.Log]::FileOnly($line) }
    else { Add-Content -Path $LogPath -Value $line }
}
function Say([string]$msg, [string]$color = 'Gray') { Write-Host $msg -ForegroundColor $color; Write-Log $msg }

Say ("Pimax dump tool v" + $ToolVersion + " - read-only. Output: " + $OutDir) 'Cyan'

try {
    Add-Type -TypeDefinition $cs -Language CSharp -ErrorAction Stop
} catch {
    Say ("The inline C# did not compile: " + $_.Exception.Message) 'Red'
    if (-not $NoPause) { Read-Host 'Press Enter to close' }
    exit 2
}
[PimaxDump.Log]::Init($LogPath)
[PimaxDump.Log]::Info("tool version $ToolVersion, PowerShell $($PSVersionTable.PSVersion), 64-bit process: $([Environment]::Is64BitProcess)")

# ---- which parts?
$sel = @()
if ($Parts -ne '') {
    $sel = @($Parts -split '[,\s]+' | Where-Object { $_ -ne '' })
} elseif ($NoPause) {
    $sel = @('1','3','4','2')
} else {
    Write-Host ''
    Write-Host 'What to collect? (all of it is read-only)'
    Write-Host '  1  distortion + FOV + eye poses + hidden-area mesh   (needs Pimax Play running, headset on)'
    Write-Host '  3  other info: IPD, device properties, camera calibration, config keys   (same)'
    Write-Host '  4  copy Pimax data files + list of dll/exe/sys with versions and SHA-256  (no headset needed)'
    Write-Host "  2  pose logger: head + controllers + input + gaze to CSV for $Seconds s   (run last)"
    $ans = Read-Host 'Enter = everything, or type numbers like "1 3"'
    if ($ans.Trim() -eq '') { $sel = @('1','3','4','2') } else { $sel = @($ans -split '[,\s]+' | Where-Object { $_ -ne '' }) }
}
Say ("parts: " + ($sel -join ',')) 'Gray'

# ---- part 0: environment snapshot (always)
try {
    $env0 = Join-Path $OutDir 'system-info.txt'
    $o = New-Object System.Collections.Generic.List[string]
    $o.Add("date: " + (Get-Date -Format o) + "   utc: " + [DateTime]::UtcNow.ToString('o'))
    $o.Add("tool version: $ToolVersion")
    $o.Add("os: " + [Environment]::OSVersion.VersionString + "   machine: " + $env:COMPUTERNAME)
    try { Get-CimInstance Win32_VideoController | ForEach-Object { $o.Add("gpu: " + $_.Name + "  driver " + $_.DriverVersion) } } catch {}
    $o.Add("--- Pimax/Tobii/SteamVR related processes")
    Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -match 'pi_|pimax|pi[A-Z]|tobii|vrserver|vrcompositor|steam' } |
        ForEach-Object { $o.Add("  " + $_.ProcessName + "  pid " + $_.Id + "  " + $_.Path) }
    $o.Add("--- services")
    Get-Service -ErrorAction SilentlyContinue | Where-Object { $_.Name -match 'pi|pimax|tobii' -and $_.DisplayName -match 'pimax|tobii|pi ?service|pvr' } |
        ForEach-Object { $o.Add("  " + $_.Name + " [" + $_.Status + "] " + $_.DisplayName) }
    $o.Add("--- present USB devices from Pimax (VID_34A4) / Tobii (VID_2104) / generic hubs")
    try {
        Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -match 'VID_34A4|VID_2104' } |
            ForEach-Object { $o.Add("  " + $_.InstanceId + "   " + $_.FriendlyName + "   [" + $_.Class + "]") }
    } catch {}
    $o.Add("--- USBPcap present?")
    $o.Add("  " + (Test-Path 'C:\Program Files\USBPcap\USBPcapCMD.exe') + "  (C:\Program Files\USBPcap\USBPcapCMD.exe)")
    $o.Add("  Wireshark: " + (Test-Path 'C:\Program Files\Wireshark\Wireshark.exe'))
    Set-Content -Path $env0 -Value $o -Encoding UTF8
    Say "wrote system-info.txt" 'Gray'
} catch { Say ("system-info failed: " + $_.Exception.Message) 'Yellow' }

# ---- locate libPVRClient64.dll
function Find-PvrDll {
    if ($DllPath -ne '' -and (Test-Path $DllPath)) { return (Resolve-Path $DllPath).Path }
    $names = @('libPVRClient64.dll', 'libpvrclient64.dll')
    $cands = New-Object System.Collections.Generic.List[string]
    foreach ($d in @("$env:ProgramFiles\Pimax\Runtime", "$env:ProgramFiles\Pimax", "${env:ProgramFiles(x86)}\Pimax", "$env:ProgramData\Pimax",
                     "$env:SystemRoot\System32", "$env:LOCALAPPDATA\Pimax")) {
        foreach ($n in $names) { $p = Join-Path $d $n; if (Test-Path $p) { $cands.Add($p) } }
    }
    foreach ($p in ($env:PATH -split ';')) { if ($p) { foreach ($n in $names) { $q = Join-Path $p $n; if (Test-Path $q) { $cands.Add($q) } } } }
    if ($cands.Count -eq 0) {
        Say 'searching Program Files\Pimax for libPVRClient64.dll (can take a minute) ...' 'Gray'
        foreach ($root in @("$env:ProgramFiles\Pimax", "${env:ProgramFiles(x86)}\Pimax")) {
            if (Test-Path $root) {
                Get-ChildItem -Path $root -Recurse -Force -ErrorAction SilentlyContinue -Filter 'libPVRClient*.dll' | ForEach-Object { $cands.Add($_.FullName) }
            }
        }
    }
    if ($cands.Count -eq 0) { return $null }
    # newest first
    return ($cands | Sort-Object { (Get-Item $_).LastWriteTime } -Descending | Select-Object -First 1)
}

$needHeadset = ($sel -contains '1') -or ($sel -contains '2') -or ($sel -contains '3')
$connected = $false
if ($needHeadset) {
    $dll = Find-PvrDll
    if (-not $dll) {
        Say 'Could not find libPVRClient64.dll. Is Pimax Play installed? Re-run with -DllPath "C:\...\libPVRClient64.dll".' 'Red'
        Say 'Fallback: look for *PVRClient*.dll under C:\Program Files\Pimax and C:\Windows\System32; send that path (and the file list in pimax-file-inventory.csv from part 4).' 'Yellow'
    } else {
        Say ("PVR client: " + $dll) 'Green'
        try { $connected = [PimaxDump.Tool]::Connect($dll, 60) } catch { Say ("connect exception: " + $_.Exception.ToString()) 'Red' }
        if (-not $connected) { Say 'No PVR session; parts 1, 2 and 3 are skipped. Check: Pimax Play running, headset connected and on.' 'Red' }
    }
}

# ---- part 1
if (($sel -contains '1') -and $connected) {
    try { [void][PimaxDump.Tool]::DumpDistortion($OutDir, $GridN, $LineN, $ToolVersion) } catch { Say ("part 1 failed: " + $_.Exception.ToString()) 'Red' }
}
# ---- part 3
if (($sel -contains '3') -and $connected) {
    try { [void][PimaxDump.Tool]::DumpInfo($OutDir, $null) } catch { Say ("part 3 failed: " + $_.Exception.ToString()) 'Red' }
}

# ---- part 4: file inventory + copy
function Test-Skip([string]$full) {
    return ($full -match '\\(node_modules|locales|translations|qml|imageformats|platforms|iconengines|styles|\$Recycle\.Bin|CrashDumps|Cache|GPUCache|Code Cache|Crashpad)\\') -or
           ($full -match '(?i)token|cookie|credential|password|passwd|toolkit|accountinfo|account_info|login|session\.json|secret|wallet')
}
if ($sel -contains '4') {
    Say 'part 4: inventory of dll/exe/sys (names, versions, SHA-256; the binaries themselves are NOT copied)' 'Cyan'
    $inv = New-Object System.Collections.Generic.List[object]
    foreach ($root in @("$env:ProgramFiles\Pimax", "${env:ProgramFiles(x86)}\Pimax", "$env:ProgramFiles\Tobii", "${env:ProgramFiles(x86)}\Tobii",
                        "$env:ProgramData\Tobii", "$env:ProgramFiles\Tobii Dynavox", "$env:ProgramFiles\Common Files\Tobii")) {
        if (-not (Test-Path $root)) { continue }
        Say ("  scanning " + $root) 'Gray'
        Get-ChildItem -Path $root -Recurse -Force -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Extension -match '^\.(dll|exe|sys)$' } | ForEach-Object {
                $h = ''
                try { $h = (Get-FileHash -Algorithm SHA256 -Path $_.FullName -ErrorAction Stop).Hash } catch {}
                $vi = $_.VersionInfo
                $inv.Add([pscustomobject]@{
                    Path = $_.FullName; SizeBytes = $_.Length; LastWriteUtc = $_.LastWriteTimeUtc.ToString('o');
                    FileVersion = $vi.FileVersion; ProductVersion = $vi.ProductVersion; ProductName = $vi.ProductName; Company = $vi.CompanyName;
                    Sha256 = $h })
            }
    }
    $inv | Export-Csv -Path (Join-Path $OutDir 'pimax-file-inventory.csv') -NoTypeInformation -Encoding UTF8
    Say ("  inventory: " + $inv.Count + " binaries") 'Green'

    Say 'part 4: copying data files (no dll/exe/sys), this can take a few minutes' 'Cyan'
    $dataExt = '^\.(yaml|yml|json|xml|ini|cfg|conf|config|bin|dat|txt|csv|toml|lut|obj|mesh|voc|db|dfu|fw|hex|calib|cal|map|log|md|plist|properties|ply|stl|tflite|onnx|bmp|png)$'
    $skipExt = '^\.(dll|exe|sys|msi|pdb|dmp|lib|obj\.meta|cat|ocx|drv)$'
    $maxFile = 200MB; $maxTotal = 4GB; $total = 0L
    $copyDirs = @(
        @{ Src = "$env:ProgramData\Pimax"; Dst = 'ProgramData-Pimax' },
        @{ Src = "$env:LOCALAPPDATA\Pimax"; Dst = 'LocalAppData-Pimax' },
        @{ Src = "$env:APPDATA\Pimax"; Dst = 'AppData-Pimax' },
        @{ Src = "$env:ProgramFiles\Pimax"; Dst = 'ProgramFiles-Pimax'; InstallDir = $true },
        @{ Src = "${env:ProgramFiles(x86)}\Pimax"; Dst = 'ProgramFilesX86-Pimax'; InstallDir = $true },
        @{ Src = "$env:ProgramData\Tobii"; Dst = 'ProgramData-Tobii' },
        @{ Src = "$env:LOCALAPPDATA\Tobii"; Dst = 'LocalAppData-Tobii' }
    )
    $copyLog = New-Object System.Collections.Generic.List[object]
    foreach ($cd in $copyDirs) {
        if (-not (Test-Path $cd.Src)) { Say ("  (absent) " + $cd.Src) 'DarkGray'; continue }
        Say ("  " + $cd.Src) 'Gray'
        $n = 0
        Get-ChildItem -Path $cd.Src -Recurse -Force -File -ErrorAction SilentlyContinue | ForEach-Object {
            $f = $_
            $why = ''
            if ($f.Extension -match $skipExt) { $why = 'binary' }
            elseif (Test-Skip $f.FullName) { $why = 'excluded-name-or-dir' }
            elseif ($f.Length -gt $maxFile) { $why = 'too-big' }
            elseif ($cd.InstallDir -and ($f.Extension -notmatch $dataExt)) { $why = 'not-data-ext' }
            elseif ($cd.InstallDir -and $f.Extension -match '^\.(png|bmp|log)$') { $why = 'ui-asset' }
            if ($why -eq '' -and ($total + $f.Length) -gt $maxTotal) { $why = 'total-cap' }
            if ($why -eq '') {
                $rel = $f.FullName.Substring($cd.Src.Length).TrimStart('\')
                $dst = Join-Path (Join-Path (Join-Path $OutDir 'files') $cd.Dst) $rel
                try {
                    New-Item -ItemType Directory -Force -Path (Split-Path $dst) | Out-Null
                    Copy-Item -LiteralPath $f.FullName -Destination $dst -Force -ErrorAction Stop
                    $total += $f.Length; $n++
                    $copyLog.Add([pscustomobject]@{ Path = $f.FullName; SizeBytes = $f.Length; Result = 'copied' })
                } catch { $copyLog.Add([pscustomobject]@{ Path = $f.FullName; SizeBytes = $f.Length; Result = ('error: ' + $_.Exception.Message) }) }
            } else {
                $copyLog.Add([pscustomobject]@{ Path = $f.FullName; SizeBytes = $f.Length; Result = ('skipped: ' + $why) })
            }
        }
        Say ("    copied " + $n + " files") 'Green'
    }
    $copyLog | Export-Csv -Path (Join-Path $OutDir 'pimax-copy-log.csv') -NoTypeInformation -Encoding UTF8
    Say ("  copied " + [math]::Round($total / 1MB, 1) + " MB in total (see pimax-copy-log.csv for skipped files)") 'Green'
}

# ---- part 2: pose logger (last)
if (($sel -contains '2') -and $connected) {
    Say '' 'Gray'
    Say "POSE LOGGER: $Seconds seconds." 'Cyan'
    Say 'If you also want the USB traffic for the same time span: start USBPcap NOW (see README, section "SLAM ground truth").' 'Yellow'
    Say 'Put the headset on, stand in the room, and be ready to walk around slowly for the whole time (look around, move, turn).' 'Yellow'
    if (-not $NoPause) { [void](Read-Host 'Press Enter to start (a 5 s countdown follows)') }
    for ($i = 5; $i -ge 1; $i--) { Write-Host -NoNewline ("starting in $i ...`r"); Start-Sleep 1 }
    try { [console]::Beep(880, 300) } catch {}
    try { [void][PimaxDump.Tool]::LogPoses($OutDir, $Seconds) } catch { Say ("part 2 failed: " + $_.Exception.ToString()) 'Red' }
    try { [console]::Beep(440, 600) } catch {}
    Say 'POSE LOGGER DONE - stop USBPcap now.' 'Green'
} elseif ($sel -contains '2') {
    Say 'pose logger skipped (no PVR session).' 'Yellow'
}

if ($connected) { [PimaxDump.Tool]::Close() }

# ---- summary
Say '' 'Gray'
Say 'Done. Files:' 'Cyan'
Get-ChildItem -Path $OutDir -File | ForEach-Object { Say ("  " + $_.Name + "  " + [math]::Round($_.Length / 1KB) + " KB") 'Gray' }
if (Test-Path (Join-Path $OutDir 'files')) {
    $sz = (Get-ChildItem -Path (Join-Path $OutDir 'files') -Recurse -File -ErrorAction SilentlyContinue | Measure-Object Length -Sum).Sum
    Say ("  files\  (copied data files) " + [math]::Round($sz / 1MB, 1) + " MB") 'Gray'
}
Say ("Copy the whole folder back: " + $OutDir) 'Green'
if (-not $NoPause) { Read-Host 'Press Enter to close' }
