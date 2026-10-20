# SPDX-License-Identifier: MIT
# Adapted from omarchyvr windows/crystal-eye-capture.ps1, capture-session.ps1, usbpcap-capture.ps1, crystal-eye-files.ps1 and evo-screens.ps1 (MIT OR Apache-2.0).
#
# PiBridge headset data kit: ONE guided run (about 10-15 minutes) that records what the official Pimax client does with
# your headset, so PiBridge can support your model on Linux. Start it with PIBRIDGE-HEADSET-DATA.cmd (it asks for
# Administrator once, USBPcap needs it).
#
# READ ONLY towards the headset: this script never sends anything to the headset, the controllers or the eye tracker.
# USBPcap only listens to the USB traffic; the files are copied, never changed; the PVR dump (kit\pimax-pvr-dump.ps1)
# only calls "get" functions of Pimax's own client library. Nothing is installed, nothing is uploaded.
#
# Every step: the script shows (and says) what to do, you do it and press Enter. Keys at every prompt:
#   Enter = done / next      s = skip this step      q = finish now (the bundle is made from what was recorded)
# The time of every Enter goes into captures\<stamp>-<part>-markers.txt. Times shown are hints only.
#
# Output: <this folder>\pibridge-headset-data-<model>-<stamp>\ and the same as a .zip next to it.
# Written for Windows PowerShell 5.1, no modules.
param(
    [string]$OutRoot = '',     # default: the folder of this script
    [switch]$NoSpeech
)
$ErrorActionPreference = 'Continue'
$KitVersion = '1'
if (-not $OutRoot) { $OutRoot = $PSScriptRoot }
if (-not $OutRoot) { $OutRoot = (Get-Location).Path }
$Inv = [Globalization.CultureInfo]::InvariantCulture

function Close-Window { [void](Read-Host 'Press Enter to close this window') }

# ------------------------------------------------------------------ checks before anything is recorded
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host 'Please start PIBRIDGE-HEADSET-DATA.cmd (double-click it). It asks for Administrator once: USBPcap needs it.' -ForegroundColor Red
    Close-Window; exit 1
}
$usbcmd = Join-Path $env:ProgramFiles 'USBPcap\USBPcapCMD.exe'
if (-not (Test-Path $usbcmd)) {
    Write-Host ''
    Write-Host 'USBPcap is not installed.' -ForegroundColor Red
    Write-Host 'Please install the official USBPcap (https://desowin.org/usbpcap/, or tick "Install USBPcap" in the'
    Write-Host 'Wireshark installer), restart Windows once, then double-click PIBRIDGE-HEADSET-DATA.cmd again.'
    Write-Host 'Nothing was recorded.'
    Close-Window; exit 1
}
$ifaces = @(& $usbcmd --extcap-interfaces 2>$null | Select-String -Pattern 'value=(\\\\\.\\USBPcap\d+)' | ForEach-Object { $_.Matches[0].Groups[1].Value })
if ($ifaces.Count -eq 0) {
    Write-Host 'USBPcap is installed but shows no USB hubs. Restart Windows once (that loads its driver), then try again.' -ForegroundColor Red
    Close-Window; exit 1
}

# ------------------------------------------------------------------ voice and screen
$voice = $null
if (-not $NoSpeech) { try { Add-Type -AssemblyName System.Speech; $voice = New-Object System.Speech.Synthesis.SpeechSynthesizer } catch { $voice = $null } }
function Speak([string]$t) {
    if (-not $voice -or -not $t) { return }
    try { $voice.SpeakAsyncCancelAll(); [void]$voice.SpeakAsync($t) } catch {}
}
function Wrap-Text([string]$t, [int]$w = 74) {
    $out = New-Object System.Collections.Generic.List[string]
    foreach ($para in ($t -split "`n")) {
        $line = ''
        foreach ($word in ($para -split ' ')) {
            if ($word -eq '') { continue }
            if ($line.Length -gt 0 -and ($line.Length + 1 + $word.Length) -gt $w) { $out.Add($line); $line = $word }
            elseif ($line.Length -gt 0) { $line += ' ' + $word }
            else { $line = $word }
        }
        $out.Add($line)
    }
    $out
}
$script:LogFile = $null
function Log([string]$t) {
    if ($script:LogFile) { try { Add-Content -LiteralPath $script:LogFile -Value ((Get-Date -Format o) + ' ' + $t) } catch {} }
}
function Say([string]$t, [string]$color = 'Gray') { Write-Host $t -ForegroundColor $color; Log $t }
# A step on screen: a coloured title bar and the text in short lines. The text is also spoken.
function Show-Big([string]$title, [string]$text, [string]$color = 'Yellow', [switch]$Quiet) {
    $bar = '=' * 78
    Write-Host ''
    Write-Host $bar -ForegroundColor $color
    Write-Host ('  ' + $title.ToUpper()) -ForegroundColor Black -BackgroundColor $color
    Write-Host $bar -ForegroundColor $color
    foreach ($l in (Wrap-Text $text)) { Write-Host ('  ' + $l) -ForegroundColor White }
    Write-Host ''
    Log ("[$title] $text")
    if (-not $Quiet) { Speak $text }
}
# One prompt. Returns 'q', 's', '' (Enter) or the typed text.
$KeysHelp = 'Enter = done / next    s = skip this step    q = finish now'
function Read-Key([string]$hint = $KeysHelp) {
    $a = Read-Host $hint
    if ($null -eq $a) { $a = '' }
    $a = $a.Trim()
    if ($a -ieq 'q') { return 'q' }
    if ($a -ieq 's') { return 's' }
    return $a
}
function Ask-YesNo([string]$q) {
    while ($true) {
        $a = (Read-Host ($q + '   y = yes, n = no, Enter = not sure')).Trim().ToLower()
        if ($a -eq 'y' -or $a -eq 'n' -or $a -eq '') { if ($a -eq '') { return '?' } else { return $a } }
    }
}
function Slug([string]$s) { ($s.Trim().ToLower() -replace '[^a-z0-9]+', '-').Trim('-') }
function Fmt([double]$x) { $x.ToString('F2', $Inv) }

# ------------------------------------------------------------------ introduction
Clear-Host
Show-Big 'PiBridge: help us support your headset' ("This records what the Pimax client does with your headset, so PiBridge can drive it on Linux. " +
    "About 10 to 15 minutes. You do each step yourself and press Enter.`n" +
    "It never sends anything to the headset and never uploads anything: all files stay in this folder until you send them.`n" +
    "Please: SteamVR and games closed, the Pimax client installed, the headset connected, controllers charged.`n" +
    "Keys at every question: Enter = done / next, s = skip this step, q = finish now.") 'Cyan'
$a = Read-Key 'Enter = start    q = quit'
if ($a -eq 'q') { exit 0 }

# ------------------------------------------------------------------ which headset, which features
$models = @(
    @{ K = '1'; Name = 'Crystal (original)';                     Slug = 'crystal' },
    @{ K = '2'; Name = 'Crystal Super';                          Slug = 'crystal-super' },
    @{ K = '3'; Name = 'Crystal Light';                          Slug = 'crystal-light' },
    @{ K = '4'; Name = 'Dream Air';                              Slug = 'dream-air' },
    @{ K = '5'; Name = 'Dream Air SE';                           Slug = 'dream-air-se' },
    @{ K = '6'; Name = '8KX';                                    Slug = '8kx' },
    @{ K = '7'; Name = '5K family (5K Plus, 5K XR, 5K Super)';   Slug = '5k' },
    @{ K = '8'; Name = '8K family (8K, 8K Plus)';                Slug = '8k' },
    @{ K = '9'; Name = 'another model (you type its name)';      Slug = '' }
)
Show-Big 'Which headset?' 'Type the number of your headset and press Enter.' 'Cyan'
foreach ($m in $models) { Write-Host ('   {0}  {1}' -f $m.K, $m.Name) }
$model = $null
while (-not $model) {
    $k = (Read-Host 'Number').Trim()
    $model = $models | Where-Object { $_.K -eq $k } | Select-Object -First 1
}
$modelName = $model.Name; $modelSlug = $model.Slug
if ($model.K -eq '9') {
    $t = ''
    while (-not (Slug $t)) { $t = Read-Host 'The model name (as Pimax calls it)' }
    $modelName = $t.Trim(); $modelSlug = Slug $t
}
if ($model.K -eq '2') {
    Write-Host '   Which Crystal Super?  1 = 57 PPD (QLED)   2 = 50 PPD (QLED)   3 = UW (ultra wide)   4 = Micro-OLED   Enter = not sure'
    $v = (Read-Host 'Number').Trim()
    $variants = @{ '1' = '57 PPD'; '2' = '50 PPD'; '3' = 'UW'; '4' = 'Micro-OLED' }
    if ($variants.ContainsKey($v)) { $modelName = 'Crystal Super ' + $variants[$v]; $modelSlug = Slug $modelName }
}
$shownName = (Read-Host 'The exact name the Pimax client shows for it, if you like (Enter = skip)').Trim()

Show-Big 'What does your headset have?' 'A few questions, so the script only asks for what your headset can do. When not sure, press Enter: you can still skip a step later.' 'Cyan' -Quiet
$F = [ordered]@{}
$F.eye         = Ask-YesNo 'Eye tracking?'
$F.controllers = Ask-YesNo 'Pimax controllers (not Valve Index controllers)?'
$F.lighthouse  = Ask-YesNo 'Can the client switch tracking between inside-out and Lighthouse (SteamVR base stations)? (The original Crystal can.)'
$F.ipd         = Ask-YesNo 'An IPD adjustment (a wheel on the headset, or set in the client)?'
$tr = (Read-Host 'Which tracking is on right now?   i = inside-out (cameras on the headset), l = Lighthouse, Enter = not sure').Trim().ToLower()
$F.tracking_now = if ($tr -eq 'i') { 'inside-out' } elseif ($tr -eq 'l') { 'lighthouse' } else { 'not sure' }
function Has([string]$k) { $F[$k] -ne 'n' }

# ------------------------------------------------------------------ the bundle folder
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$bundle = Join-Path $OutRoot ("pibridge-headset-data-{0}-{1}" -f $modelSlug, $stamp)
$capDir = Join-Path $bundle 'captures'
$invDir = Join-Path $bundle 'inventory'
New-Item -ItemType Directory -Force $capDir, $invDir | Out-Null
$script:LogFile = Join-Path $bundle 'run.log'
Log ("kit version $KitVersion, model '$modelName', client name '$shownName', PowerShell " + $PSVersionTable.PSVersion + ", hubs " + $ifaces.Count)
$script:Quit = $false
$script:Done = New-Object System.Collections.Generic.List[string]     # "part: result" lines for README.txt
$script:Serials = New-Object 'System.Collections.Generic.HashSet[string]'

try {
    $free = (Get-PSDrive -Name $bundle.Substring(0, 1) -ErrorAction Stop).Free / 1GB
    Say ("Free space on this drive: {0} GB. The recordings can need up to 10 GB." -f $free.ToString('F1', $Inv))
    if ($free -lt 10) {
        Show-Big 'Little free space' 'This drive has less than 10 GB free. Better: copy this folder to a drive with more space and start again. Enter = go on anyway, q = stop.' 'Red'
        if ((Read-Key 'Enter = go on    q = stop') -eq 'q') { $script:Quit = $true }
    }
} catch {}

# ------------------------------------------------------------------ USB recording (USBPcap on every root hub, whole packets)
function Start-Capture([string]$take) {
    $st = Get-Date -Format 'yyyyMMdd-HHmmss'
    $c = @{ Take = $take; Files = @(); Procs = @(); Markers = (Join-Path $capDir "$st-$take-markers.txt"); T0 = $null }
    foreach ($i in $ifaces) {
        $n = ($i -split 'USBPcap')[1]
        $f = Join-Path $capDir ("{0}-{1}-hub{2}.pcap" -f $st, $take, $n)
        $c.Files += $f
        $c.Procs += Start-Process -FilePath $usbcmd -ArgumentList "-d $i -A -s 65535 -o `"$f`"" -WindowStyle Hidden -PassThru
    }
    Start-Sleep -Milliseconds 1500    # every hub's recording is running before the first step
    $alive = @($c.Procs | Where-Object { -not $_.HasExited }).Count
    if ($alive -eq 0) { Say 'USBPcap did not start on any hub. Restart Windows once and run the kit again.' Red }
    $c.T0 = Get-Date
    Add-Content $c.Markers ("start {0:o} utc {1:o} part {2} hubs {3}" -f $c.T0, $c.T0.ToUniversalTime(), $take, $alive)
    Say ("RECORDING '{0}' on {1} hub(s)" -f $take, $alive) Green
    $c
}
function Mark($c, [string]$what) {
    $now = Get-Date
    Add-Content $c.Markers ("{0,9} s  {1}  {2:o}" -f (Fmt ($now - $c.T0).TotalSeconds), $what, $now)
}
function Stop-Capture($c) {
    Mark $c 'stop'
    $c.Procs | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 800
    $mb = 0
    foreach ($f in $c.Files) {
        if (-not (Test-Path $f)) { continue }
        $len = (Get-Item $f).Length
        if ($len -lt 256) { Remove-Item $f -Force; continue }    # a hub with nothing on it
        $mb += $len / 1MB
    }
    Say ("STOPPED '{0}': {1} MB recorded" -f $c.Take, $mb.ToString('F0', $Inv)) Green
}

# One recorded part: an intro (Enter starts the recording), then its steps, each ended by Enter.
# A step: @{ M = marker name; Show = text (also spoken); Min = hint in s (an Enter before it asks once more);
#            Max = hint in s; Free = $true (each typed line is a marker, Enter alone ends the step);
#            Check = scriptblock that returns a message when the step does not look done (asks once more) }
function Run-Part([string]$take, [string]$title, [string]$intro, $steps) {
    if ($script:Quit) { return $null }
    Show-Big $title $intro 'Cyan'
    $a = Read-Key 'Enter = start recording    s = skip this part    q = finish now'
    if ($a -eq 'q') { $script:Quit = $true; return $null }
    if ($a -eq 's') { $script:Done.Add("$title : skipped"); Log "skipped part $take"; return $null }
    $c = Start-Capture $take
    $steps = @($steps)
    $i = 0; $skipped = 0
    foreach ($s in $steps) {
        $i++
        $hint = ''
        if ($s.Min -and $s.Max) { $hint = " (about $($s.Min) to $($s.Max) seconds)" }
        elseif ($s.Min) { $hint = " (at least $($s.Min) seconds)" }
        elseif ($s.Max) { $hint = " (up to $($s.Max) seconds)" }
        Show-Big ("{0}: step {1} of {2}" -f $title, $i, $steps.Count) ($s.Show + $hint) 'Yellow'
        Mark $c ('begin ' + $s.M)
        $t0 = Get-Date
        $result = 'done'
        if ($s.Free) {
            while ($true) {
                $a = Read-Key 'type what you did + Enter = a marker    Enter alone = done    s = skip    q = finish now'
                if ($a -eq '') { break }
                if ($a -eq 's' -or $a -eq 'q') { $result = $a; break }
                Mark $c ('note ' + $a); Write-Host '   marked' -ForegroundColor DarkGray
            }
        } else {
            $asked = $false; $checked = $false
            while ($true) {
                $a = Read-Key
                if ($a -eq 's' -or $a -eq 'q') { $result = $a; break }
                if ($a -ne '') { Mark $c ('note ' + $a) }
                $el = ((Get-Date) - $t0).TotalSeconds
                if ($s.Min -and $el -lt $s.Min -and -not $asked) {
                    $asked = $true
                    Write-Host ("   That was {0} s; this step needs about {1} s. Wait a little, then Enter again." -f [int]$el, $s.Min) -ForegroundColor Yellow
                    continue
                }
                if ($s.Check -and -not $checked) {
                    $checked = $true
                    $msg = & $s.Check
                    if ($msg) { Write-Host ('   ' + $msg) -ForegroundColor Yellow; Mark $c ('check ' + $s.M + ': not done yet'); continue }
                }
                break
            }
        }
        if ($result -eq 'q') { Mark $c ('quit ' + $s.M); $script:Quit = $true; break }
        if ($result -eq 's') { Mark $c ('skip ' + $s.M); $skipped++; continue }
        Mark $c ('done ' + $s.M)
        Write-Host '   OK' -ForegroundColor Green
    }
    Stop-Capture $c
    if (-not $script:Quit) {
        $n = (Read-Host 'Anything to note about this part (for example something that looked different)? Type it, or Enter').Trim()
        if ($n -ieq 'q') { $script:Quit = $true }
        elseif ($n -and $n -ine 's') { Add-Content $c.Markers ('note: ' + $n) }
    }
    $script:Done.Add(("{0} : recorded, {1} of {2} steps skipped" -f $title, $skipped, $steps.Count))
    return $c
}

# The client programs (not its services): PimaxClient (Pimax Play / Pimax EVO), older PiTool.
function Get-ClientProcs { @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -match '^(PimaxClient|PimaxPlay|PimaxEVO|PiTool|PiPlay)' }) }
$CheckClosed = { if ((Get-ClientProcs).Count -gt 0) { 'The Pimax client still runs. Close it fully (also the tray icon: right-click it, Exit), then Enter again.' } }

# ------------------------------------------------------------------ serial masking (our own text files only)
function Mask-Id([string]$id) {
    if (-not $id) { return $id }
    $parts = $id -split '\\'
    if ($parts.Count -ge 3) {
        $last = $parts[$parts.Count - 1]
        # a USB instance id without '&' is the device's serial string; ids with '&' are made by Windows from the port
        if ($last -notmatch '&') {
            if ($last.Length -ge 5) { [void]$script:Serials.Add($last) }
            $parts[$parts.Count - 1] = 'SERIAL-MASKED'
        }
    }
    $parts -join '\'
}
function Scrub-Dir([string]$dir) {
    if (-not (Test-Path $dir)) { return }
    Get-ChildItem -LiteralPath $dir -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { $_.Extension -match '^\.(txt|json|csv|log|xml|ini|cfg|conf|yaml|yml)$' -and $_.Length -lt 200MB } | ForEach-Object {
            $file = $_.FullName
            try {
                $txt = [IO.File]::ReadAllText($file)
                $new = $txt
                # Windows writes USB serials in upper case, the files may not: compare without case
                foreach ($s in $script:Serials) { $new = [regex]::Replace($new, '(?i)' + [regex]::Escape($s), 'SERIAL-MASKED') }
                if ($env:COMPUTERNAME -and $env:COMPUTERNAME.Length -ge 3) { $new = [regex]::Replace($new, '(?i)\b' + [regex]::Escape($env:COMPUTERNAME) + '\b', 'THIS-PC') }
                if ($env:USERNAME) { $new = [regex]::Replace($new, '(?i)(\\\\|\\|/)Users(\\\\|\\|/)' + [regex]::Escape($env:USERNAME) + '(?=\\|/|"|$)', '$1Users$2USER') }
                if ($new -cne $txt) { [IO.File]::WriteAllText($file, $new) }
            } catch { Log ("could not mask " + $file) }
        }
}

# ------------------------------------------------------------------ part 1: inventory (read only, no recording)
function Collect-Inventory {
    Say 'Reading versions and USB devices ...' Cyan
    $o = New-Object System.Collections.Generic.List[string]
    $o.Add("PiBridge headset data kit $KitVersion, " + (Get-Date -Format o))
    $o.Add("model (chosen): $modelName    shown by the client: $shownName")
    foreach ($k in $F.Keys) { $o.Add(("feature {0}: {1}" -f $k, $F[$k])) }
    try { $os = Get-CimInstance Win32_OperatingSystem; $o.Add("windows: " + $os.Caption + " " + $os.Version + " build " + $os.BuildNumber) } catch {}
    $o.Add("powershell: " + $PSVersionTable.PSVersion)
    try { Get-CimInstance Win32_VideoController | ForEach-Object { $o.Add("gpu: " + $_.Name + "  driver " + $_.DriverVersion + "  " + $_.CurrentHorizontalResolution + "x" + $_.CurrentVerticalResolution + " @" + $_.CurrentRefreshRate) } } catch {}
    $o.Add("usbpcap: " + (Get-Item $usbcmd).VersionInfo.FileVersion + "   hubs: " + $ifaces.Count)
    $o.Add('')
    $o.Add('--- installed programs (Pimax, Tobii, SteamVR, USBPcap)')
    $keys = @('HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*', 'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*', 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*')
    Get-ItemProperty $keys -ErrorAction SilentlyContinue | Where-Object { $_.DisplayName -match 'pimax|tobii|steamvr|usbpcap|pitool' } |
        Sort-Object DisplayName | ForEach-Object { $o.Add(("  {0}  {1}  ({2})" -f $_.DisplayName, $_.DisplayVersion, $_.Publisher)) }
    $o.Add('')
    $o.Add('--- program files of Pimax and Tobii (exe only: name and version)')
    foreach ($root in @("$env:ProgramFiles\Pimax", "${env:ProgramFiles(x86)}\Pimax", "$env:ProgramFiles\Tobii", "${env:ProgramFiles(x86)}\Tobii", "$env:ProgramFiles\PiTool")) {
        if (-not (Test-Path $root)) { continue }
        Get-ChildItem -LiteralPath $root -Recurse -File -Filter '*.exe' -ErrorAction SilentlyContinue | ForEach-Object {
            $o.Add(("  {0}  file {1}  product {2}" -f $_.FullName, $_.VersionInfo.FileVersion, $_.VersionInfo.ProductVersion))
        }
    }
    $o.Add('')
    $o.Add('--- services (Pimax, Tobii)')
    try {
        Get-CimInstance Win32_Service | Where-Object { $_.Name -match 'pimax|tobii|piservice|pvr' -or $_.DisplayName -match 'pimax|tobii' } | ForEach-Object {
            $exe = ($_.PathName -replace '^"([^"]+)".*$', '$1') -replace '^(\S+\.exe).*$', '$1'
            $ver = ''; try { $ver = (Get-Item -LiteralPath $exe -ErrorAction Stop).VersionInfo.FileVersion } catch {}
            $o.Add(("  {0}  [{1}, {2}]  {3}  version {4}" -f $_.Name, $_.State, $_.StartMode, $_.DisplayName, $ver))
        }
    } catch {}
    $o.Add('')
    $o.Add('--- running programs (names)')
    Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -match 'pimax|^pi|tobii|vrserver|vrcompositor|steam' } |
        Sort-Object ProcessName -Unique | ForEach-Object { $o.Add("  " + $_.ProcessName) }
    $o.Add('')
    $o.Add('--- displays (EDID names; serial numbers are not read)')
    try {
        Get-CimInstance -Namespace root\wmi -ClassName WmiMonitorID -ErrorAction Stop | ForEach-Object {
            $mf = ($_.ManufacturerName | Where-Object { $_ -ne 0 } | ForEach-Object { [char]$_ }) -join ''
            $nm = ($_.UserFriendlyName | Where-Object { $_ -ne 0 } | ForEach-Object { [char]$_ }) -join ''
            $pc = ($_.ProductCodeID | Where-Object { $_ -ne 0 } | ForEach-Object { [char]$_ }) -join ''
            $o.Add(("  {0} {1} product {2}  active {3}" -f $mf, $nm, $pc, $_.Active))
        }
    } catch {}
    Set-Content -LiteralPath (Join-Path $invDir 'software.txt') -Value $o -Encoding UTF8

    # USB devices of Pimax (34a4), Tobii (2104), Valve (28de) and older Pimax models (0483): ids and product strings
    $u = New-Object System.Collections.Generic.List[object]
    $devs = @()
    try { $devs = @(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object { $_.InstanceId -match 'VID_(34A4|2104|28DE|0483)' }) }
    catch { $devs = @(Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -match 'VID_(34A4|2104|28DE|0483)' } | ForEach-Object { [pscustomobject]@{ InstanceId = $_.DeviceID; FriendlyName = $_.Name; Class = $_.PNPClass; Status = $_.Status } }) }
    foreach ($d in $devs) {
        $p = @{}
        foreach ($k in @('DEVPKEY_Device_BusReportedDeviceDesc', 'DEVPKEY_Device_HardwareIds', 'DEVPKEY_Device_Manufacturer', 'DEVPKEY_Device_Service', 'DEVPKEY_Device_DriverVersion', 'DEVPKEY_Device_LocationInfo', 'DEVPKEY_Device_Parent')) {
            try { $p[$k] = (Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName $k -ErrorAction Stop).Data } catch { $p[$k] = $null }
        }
        $vidpid = ''; if ($d.InstanceId -match 'VID_([0-9A-F]{4})&PID_([0-9A-F]{4})') { $vidpid = ($Matches[1] + ':' + $Matches[2]).ToLower() }
        $mi = ''; if ($d.InstanceId -match '&MI_([0-9A-F]{2})') { $mi = $Matches[1] }
        $u.Add([pscustomobject]@{
            VidPid = $vidpid; Interface = $mi; Class = $d.Class; Status = $d.Status; Name = $d.FriendlyName
            UsbProductString = $p['DEVPKEY_Device_BusReportedDeviceDesc']; Manufacturer = $p['DEVPKEY_Device_Manufacturer']
            Driver = $p['DEVPKEY_Device_Service']; DriverVersion = $p['DEVPKEY_Device_DriverVersion']; Location = $p['DEVPKEY_Device_LocationInfo']
            HardwareIds = (@($p['DEVPKEY_Device_HardwareIds']) -join ' ')
            InstanceId = (Mask-Id $d.InstanceId); Parent = (Mask-Id ([string]$p['DEVPKEY_Device_Parent']))
        })
    }
    $u | Sort-Object VidPid, Interface | Export-Csv -LiteralPath (Join-Path $invDir 'usb-devices.csv') -NoTypeInformation -Encoding UTF8
    $u | Sort-Object VidPid, Interface | Format-List | Out-String -Width 300 | Set-Content -LiteralPath (Join-Path $invDir 'usb-devices.txt') -Encoding UTF8
    Say ("  {0} USB devices of Pimax / Tobii / Valve listed (serials masked)" -f $u.Count)

    # EDIDs of Pimax displays (the panel modes), serial fields cleared, checksum fixed
    $n = 0
    Get-ChildItem 'HKLM:\SYSTEM\CurrentControlSet\Enum\DISPLAY' -ErrorAction SilentlyContinue | Where-Object { $_.PSChildName -match '^(PVR|PIM|PXN)' } | ForEach-Object {
        $mon = $_.PSChildName
        Get-ChildItem $_.PSPath -ErrorAction SilentlyContinue | ForEach-Object {
            try {
                $e = (Get-ItemProperty -LiteralPath (Join-Path $_.PSPath 'Device Parameters') -Name EDID -ErrorAction Stop).EDID
                if ($e -and $e.Length -ge 128) {
                    $b = [byte[]]$e.Clone()
                    for ($j = 12; $j -le 15; $j++) { $b[$j] = 0 }                       # serial number
                    foreach ($off in 54, 72, 90, 108) {                                   # display serial string descriptor (0xFF)
                        if ($b[$off] -eq 0 -and $b[$off + 1] -eq 0 -and $b[$off + 3] -eq 0xFF) { for ($j = $off + 5; $j -lt $off + 18; $j++) { $b[$j] = 0x20 } }
                    }
                    $sum = 0; for ($j = 0; $j -lt 127; $j++) { $sum += $b[$j] }
                    $b[127] = [byte]((256 - ($sum % 256)) % 256)
                    $n++
                    [IO.File]::WriteAllBytes((Join-Path $invDir ("edid-{0}-{1}.bin" -f $mon, $n)), $b)
                }
            } catch {}
        }
    }
    if ($n -gt 0) { Say "  $n headset EDID(s) saved (serial fields cleared)" }
}

function Collect-Pvr {
    $dump = Join-Path $PSScriptRoot 'kit\pimax-pvr-dump.ps1'
    if (-not (Test-Path $dump)) { Say 'kit\pimax-pvr-dump.ps1 is missing: the PVR dump is skipped.' Yellow; return }
    $pvrDir = Join-Path $bundle 'pvr'
    New-Item -ItemType Directory -Force $pvrDir | Out-Null
    $psExe = Join-Path $env:SystemRoot 'Sysnative\WindowsPowerShell\v1.0\powershell.exe'
    if (-not (Test-Path $psExe)) { $psExe = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe' }
    Say 'Asking the Pimax client library for the display data (lens, field of view, IPD; read only, up to 2 minutes) ...' Cyan
    try {
        $p = Start-Process -FilePath $psExe -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$dump`" -Parts 1,3 -NoPause -OutRoot `"$pvrDir`"" -NoNewWindow -PassThru
        if (-not $p.WaitForExit(300000)) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue; Say 'The PVR dump took too long and was stopped.' Yellow }
    } catch { Say ('The PVR dump could not start: ' + $_.Exception.Message) Yellow }
    # the distortion file is named after the serial: rename it, remember the serial for masking
    Get-ChildItem -LiteralPath $pvrDir -Recurse -File -Filter 'pimax-distortion-*.json' -ErrorAction SilentlyContinue | ForEach-Object {
        $s = $_.BaseName.Substring('pimax-distortion-'.Length)
        if ($s -notmatch '^\d{8}-\d{6}$' -and $s.Length -ge 5) { [void]$script:Serials.Add($s) }
        Rename-Item -LiteralPath $_.FullName -NewName 'pimax-distortion.json' -ErrorAction SilentlyContinue
    }
    foreach ($j in @(Get-ChildItem -LiteralPath $pvrDir -Recurse -File -Filter '*.json' -ErrorAction SilentlyContinue)) {
        try {
            $txt = [IO.File]::ReadAllText($j.FullName)
            foreach ($mm in [regex]::Matches($txt, '"(serial|hmd_serial|hmd_sn|Serial_String)"\s*:\s*"([^"]+)"')) { if ($mm.Groups[2].Value.Length -ge 5) { [void]$script:Serials.Add($mm.Groups[2].Value) } }
            $pn = [regex]::Match($txt, '"product_name"\s*:\s*"([^"]*)"')
            if ($pn.Success -and $pn.Groups[1].Value) { $script:PvrProduct = $pn.Groups[1].Value }
        } catch {}
    }
    if ($script:PvrProduct) { Say ("  the client library says: " + $script:PvrProduct) Green }
    elseif (-not (Get-ChildItem -LiteralPath $pvrDir -Recurse -File -Filter '*.json' -ErrorAction SilentlyContinue)) { Say '  no PVR data (the client was not running or this model has no PVR library). That is fine.' Yellow }
}

# ------------------------------------------------------------------ copies (read only)
$skipExt = @('.dll', '.exe', '.sys', '.msi', '.cab', '.pdb', '.node', '.dmp')
$skipName = '(?i)token|cookie|credential|password|passwd|login|account|secret|wallet|session\.json|\\Cache\\|GPUCache|Code Cache|Crashpad|CrashDumps'
function Copy-Tree([string]$root, [string]$dest, [string]$extPattern, [long]$maxFile, $log) {
    if (-not (Test-Path $root)) { Add-Content $log "absent: $root"; return }
    Add-Content $log "copying $root"
    Get-ChildItem -LiteralPath $root -Recurse -File -Force -ErrorAction SilentlyContinue | ForEach-Object {
        if ($skipExt -contains $_.Extension.ToLower()) { return }
        if ($_.FullName -match $skipName) { Add-Content $log ("skipped (private name): " + $_.Name); return }
        if ($extPattern -and $_.Extension -notmatch $extPattern) { return }
        if ($_.Length -gt $maxFile) { Add-Content $log ("too big, skipped: {0} ({1} B)" -f $_.Name, $_.Length); return }
        $to = Join-Path $dest $_.FullName.Substring($root.Length).TrimStart('\')
        New-Item -ItemType Directory -Force (Split-Path $to) | Out-Null
        try { Copy-Item -LiteralPath $_.FullName -Destination $to -Force -ErrorAction Stop }
        catch { Add-Content $log ("could not copy {0}: {1}" -f $_.Name, $_.Exception.Message) }
    }
}
# Tobii and Pimax eye tracking files (calibration.setpm and the rest), as crystal-eye-files.ps1. NOT masked: exact copies.
function Copy-EyeFiles {
    $dest = Join-Path $bundle 'eye-files'
    New-Item -ItemType Directory -Force $dest | Out-Null
    $log = Join-Path $dest 'copy.log'
    Say 'Copying the eye tracking files (read only) ...' Cyan
    foreach ($d in (Get-ChildItem 'C:\ProgramData' -Directory -Force -ErrorAction SilentlyContinue | Where-Object { $_.Name -match 'tobii' })) {
        Copy-Tree $d.FullName (Join-Path $dest ("ProgramData\" + $d.Name)) '' 50MB $log
    }
    foreach ($base in @($env:LOCALAPPDATA, $env:APPDATA)) {
        foreach ($d in (Get-ChildItem $base -Directory -Force -ErrorAction SilentlyContinue | Where-Object { $_.Name -match 'tobii' })) {
            Copy-Tree $d.FullName (Join-Path $dest ((Split-Path $base -Leaf) + "\" + $d.Name)) '' 50MB $log
        }
    }
    # Pimax eye settings: files whose name speaks of the eyes, plus the client's settings files
    foreach ($base in @("$env:ProgramData\Pimax", "$env:LOCALAPPDATA\Pimax", "$env:APPDATA\Pimax")) {
        if (-not (Test-Path $base)) { continue }
        Get-ChildItem -LiteralPath $base -Recurse -File -Force -ErrorAction SilentlyContinue |
            Where-Object { ($_.FullName -match '(?i)eye|tobii|gaze|calib|foveat' -or $_.Extension -match '^\.(setpm)$') -and $_.FullName -notmatch $skipName -and $skipExt -notcontains $_.Extension.ToLower() -and $_.Length -lt 50MB } |
            ForEach-Object {
                $to = Join-Path (Join-Path $dest ('Pimax-' + (Split-Path (Split-Path $base) -Leaf))) $_.FullName.Substring($base.Length).TrimStart('\')
                New-Item -ItemType Directory -Force (Split-Path $to) | Out-Null
                Copy-Item -LiteralPath $_.FullName -Destination $to -Force -ErrorAction SilentlyContinue
            }
    }
    Get-ChildItem @('C:\ProgramData', $env:LOCALAPPDATA, $env:APPDATA) -Recurse -File -Force -Filter '*.setpm' -ErrorAction SilentlyContinue |
        Select-Object FullName, Length, LastWriteTime | Format-Table -AutoSize | Out-String -Width 400 | Set-Content (Join-Path $dest 'setpm-files.txt')
    $n = @(Get-ChildItem $dest -Recurse -File).Count
    Say ("  {0} files copied" -f $n) Green
}
# The client's logs and settings files (text only), as capture-session.ps1 Copy-Logs.
function Copy-ClientLogs {
    $dest = Join-Path $bundle 'client-logs'
    New-Item -ItemType Directory -Force $dest | Out-Null
    $log = Join-Path $dest 'copy.log'
    Say 'Copying the Pimax client logs and settings (text files, read only) ...' Cyan
    foreach ($s in @("$env:ProgramData\Pimax", "$env:LOCALAPPDATA\Pimax", "$env:APPDATA\Pimax")) {
        $name = (Split-Path (Split-Path $s) -Leaf) + '-' + (Split-Path $s -Leaf)
        Copy-Tree $s (Join-Path $dest $name) '^\.(log|txt|json|xml|ini|cfg|conf|yaml|yml|csv)$' 50MB $log
    }
    Scrub-Dir $dest
}

# ------------------------------------------------------------------ part 7: pictures of the client window (as evo-screens.ps1)
function Init-Screens {
    if ('PiBridgeWin' -as [type]) { return $true }
    try {
        Add-Type -AssemblyName System.Drawing
        Add-Type @"
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class PiBridgeWin {
    public delegate bool EnumProc(IntPtr h, IntPtr l);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc f, IntPtr l);
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("dwmapi.dll")] public static extern int DwmGetWindowAttribute(IntPtr h, int attr, out RECT r, int size);
    public static List<IntPtr> All() {
        var l = new List<IntPtr>();
        EnumWindows((h, x) => { l.Add(h); return true; }, IntPtr.Zero);
        return l;
    }
}
"@
        [PiBridgeWin]::SetProcessDPIAware() | Out-Null
        return $true
    } catch { Say ('Pictures are not possible here: ' + $_.Exception.Message) Yellow; return $false }
}
# The client's main window: the biggest visible titled top-level window of a Pimax client process.
function Find-ClientWindow {
    $ids = @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.ProcessName -match '^(Pimax|PiTool|PiPlay)' } | ForEach-Object { $_.Id })
    if ($ids.Count -eq 0) { return $null }
    $best = $null; $bestArea = 0
    foreach ($h in [PiBridgeWin]::All()) {
        $procId = [uint32]0; [PiBridgeWin]::GetWindowThreadProcessId($h, [ref]$procId) | Out-Null
        if ($ids -notcontains [int]$procId -or -not [PiBridgeWin]::IsWindowVisible($h)) { continue }
        $t = New-Object System.Text.StringBuilder 256; [PiBridgeWin]::GetWindowText($h, $t, 256) | Out-Null
        if (-not $t.ToString() -or $t.ToString() -match 'toast|recording') { continue }
        $r = New-Object PiBridgeWin+RECT; [PiBridgeWin]::GetWindowRect($h, [ref]$r) | Out-Null
        $area = ($r.R - $r.L) * ($r.B - $r.T)
        if ($area -gt $bestArea) { $best = $h; $bestArea = $area }
    }
    return $best
}
function Test-Blank([System.Drawing.Bitmap]$b) {
    $first = $b.GetPixel(0, 0).ToArgb()
    foreach ($fx in 0.25, 0.5, 0.75) { foreach ($fy in 0.25, 0.5, 0.75) {
        if ($b.GetPixel([int]($b.Width * $fx), [int]($b.Height * $fy)).ToArgb() -ne $first) { return $false }
    } }
    return $true
}
function Shoot-Window([IntPtr]$h, [string]$file) {
    if ([PiBridgeWin]::IsIconic($h)) { [PiBridgeWin]::ShowWindow($h, 9) | Out-Null; Start-Sleep -Milliseconds 800 }   # SW_RESTORE
    $wr = New-Object PiBridgeWin+RECT; [PiBridgeWin]::GetWindowRect($h, [ref]$wr) | Out-Null
    $vr = New-Object PiBridgeWin+RECT
    if ([PiBridgeWin]::DwmGetWindowAttribute($h, 9, [ref]$vr, 16) -ne 0) { $vr = $wr }   # DWMWA_EXTENDED_FRAME_BOUNDS
    $fw = $wr.R - $wr.L; $fh = $wr.B - $wr.T; $w = $vr.R - $vr.L; $hg = $vr.B - $vr.T
    if ($w -le 0 -or $hg -le 0 -or $fw -le 0 -or $fh -le 0) { Say '  the client window has no size (minimised?)' Yellow; return $false }
    $full = New-Object System.Drawing.Bitmap $fw, $fh
    $g = [System.Drawing.Graphics]::FromImage($full); $hdc = $g.GetHdc()
    $ok = [PiBridgeWin]::PrintWindow($h, $hdc, 2)                                         # PW_RENDERFULLCONTENT
    $g.ReleaseHdc($hdc); $g.Dispose()
    $bmp = $full.Clone((New-Object System.Drawing.Rectangle ($vr.L - $wr.L), ($vr.T - $wr.T), $w, $hg), $full.PixelFormat)
    $full.Dispose()
    if (-not $ok -or (Test-Blank $bmp)) {                                               # fallback: from the screen
        $g = [System.Drawing.Graphics]::FromImage($bmp)
        $g.CopyFromScreen($vr.L, $vr.T, 0, 0, (New-Object System.Drawing.Size $w, $hg)); $g.Dispose()
    }
    $bmp.Save($file, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
    Say ("  saved {0}" -f (Split-Path $file -Leaf)) Green
    return $true
}
function Run-Screens {
    if ($script:Quit) { return }
    $title = 'Part 7 of 7: pictures of the client'
    Show-Big $title 'Optional. Open one page of the Pimax client after the other (home, device, display, settings, controllers, eye tracking). For each page: come back here, type a short name for it and press Enter. The script saves a picture of the client window only. Enter alone = done. The device page may show the serial number: that picture is listed in PRIVATE.txt.' 'Cyan'
    $a = Read-Key 'Enter = start    s = skip this part    q = finish now'
    if ($a -eq 'q') { $script:Quit = $true; return }
    if ($a -eq 's') { $script:Done.Add("$title : skipped"); return }
    if (-not (Init-Screens)) { $script:Done.Add("$title : not possible"); return }
    $dest = Join-Path $bundle 'screens'
    New-Item -ItemType Directory -Force $dest | Out-Null
    $n = 0
    while ($true) {
        $k = Read-Key 'page name + Enter = take a picture    Enter alone = done    s = skip    q = finish now'
        if ($k -eq '' -or $k -eq 's') { break }
        if ($k -eq 'q') { $script:Quit = $true; break }
        $h = Find-ClientWindow
        if (-not $h) { Say '  no Pimax client window found: is the client open?' Yellow; continue }
        $name = Slug $k; if (-not $name) { $name = 'page' }
        if (Shoot-Window $h (Join-Path $dest ("{0:D2}-{1}.png" -f ($n + 1), $name))) { $n++ }
    }
    $script:Done.Add("$title : $n pictures")
}

# ================================================================== the run
# ---- part 1
if (-not $script:Quit) {
    $title = 'Part 1 of 7: what is installed'
    Show-Big $title 'Start the Pimax client, switch the headset on and wait until it shows the Pimax home. SteamVR stays closed. Then press Enter. The script reads versions, the USB device list and the display data (read only, about 1 minute).' 'Cyan'
    $a = Read-Key
    if ($a -eq 'q') { $script:Quit = $true }
    elseif ($a -eq 's') { $script:Done.Add("$title : skipped") }
    else {
        try { Collect-Inventory } catch { Say ('Inventory failed: ' + $_.Exception.Message) Yellow }
        try { Collect-Pvr } catch { Say ('PVR dump failed: ' + $_.Exception.Message) Yellow }
        $script:Done.Add("$title : done")
    }
}

# ---- part 2: cold start of the client
$steps = @(
    @{ M = 'client-close';  Show = 'Close the Pimax client fully: its window AND its tray icon (right-click it, Exit). Then Enter.'; Check = $CheckClosed },
    @{ M = 'closed-wait';   Show = 'Wait and do nothing for 20 seconds. Then Enter.'; Min = 20 },
    @{ M = 'client-start';  Show = 'Start the Pimax client now. Press Enter when the headset shows the Pimax home.'; Min = 15; Max = 120 }
)
if (Has 'eye') { $steps += @{ M = 'eye-ready'; Show = 'Wait until the client shows eye tracking as ready. Then Enter.'; Max = 60 } }
$steps += @{ M = 'home-idle'; Show = 'Leave everything as it is for 20 seconds. Then Enter.'; Min = 20 }
[void](Run-Part 'cold-start' 'Part 2 of 7: start of the client' 'The USB recording runs while you close the Pimax client and start it again. This shows how the client wakes the headset. About 2 minutes. Enter starts the recording.' $steps)

# ---- part 3: settings
if (-not $script:Quit) {
    $steps = @()
    Show-Big 'Refresh rates' 'Which refresh rates does the client offer for your headset? Type them with spaces, for example 72 90 120, then Enter. Enter alone: you will go through them yourself.' 'Cyan' -Quiet
    $ra = Read-Key 'Refresh rates (for example 72 90 120)    Enter alone = go through them yourself    q = finish now'
    if ($ra -eq 'q') { $script:Quit = $true }
    $rates = @(($ra -split '[,;\s]+') | Where-Object { $_ -match '^\d+(\.\d+)?$' })
    if ($rates.Count -gt 0) {
        foreach ($r in $rates) { $steps += @{ M = "rate-$r"; Show = "In the client, set the refresh rate to $r Hz. Wait until the headset shows the picture again, then Enter."; Max = 60 } }
        $steps += @{ M = 'rate-back'; Show = 'Set the refresh rate back to the one you use. Then Enter.' }
    } else {
        $steps += @{ M = 'rates'; Free = $true; Show = 'In the client, set each refresh rate it offers, one after the other. After each one type its value (for example 90) and Enter. Enter alone when done, with your usual rate set again.' }
    }
    $steps += @{ M = 'brightness-low';  Show = 'Set the brightness to the lowest value. Then Enter.' }
    $steps += @{ M = 'brightness-high'; Show = 'Set the brightness to the highest value. Then Enter.' }
    $steps += @{ M = 'brightness-back'; Show = 'Set the brightness back to your value. Then Enter.' }
    if (Has 'lighthouse') {
        $steps += @{ M = 'mode-lighthouse'; Show = 'Switch the tracking to Lighthouse (SteamVR base stations) in the client. Press Enter when the client shows the new mode.'; Max = 120 }
        $steps += @{ M = 'mode-inside-out'; Show = 'Switch the tracking back to inside-out. Press Enter when the client shows it.'; Max = 120 }
        $steps += @{ M = 'mode-back';       Show = 'If you normally use Lighthouse, switch back to Lighthouse now. Then Enter (or s).' }
    }
    if (Has 'ipd') {
        $steps += @{ M = 'ipd-small'; Show = 'Set the IPD to the smallest value (the wheel on the headset, or in the client). Then Enter.' }
        $steps += @{ M = 'ipd-large'; Show = 'Set the IPD to the largest value. Then Enter.' }
        $steps += @{ M = 'ipd-back';  Show = 'Set the IPD back to your value. Then Enter.' }
    }
    $steps += @{ M = 'audio';  Show = 'Sound: in the client or in the Windows sound settings, choose the headset as the sound output and play a sound for a few seconds. Change the volume once. Then Enter.' }
    $steps += @{ M = 'other';  Free = $true; Show = 'Other headset settings the client has (for example local dimming, colour, sharpness): change each once and back. After each, type its name and Enter. Enter alone when done.' }
    [void](Run-Part 'settings' 'Part 3 of 7: settings' 'The USB recording runs while you change the headset settings in the Pimax client, one at a time. About 3 to 5 minutes. Enter starts the recording.' $steps)
}

# ---- part 4: sleep and wake
$steps = @(
    @{ M = 'standby';        Show = 'Put the headset to sleep: standby in the client, or press the power button of the headset once. Enter when its screens are off.'; Max = 60 },
    @{ M = 'standby-wait';   Show = 'Leave it asleep for 30 seconds. Then Enter.'; Min = 30 },
    @{ M = 'wake';           Show = 'Wake the headset (in the client, the power button, or put it on). Enter when it shows the Pimax home again.'; Max = 90 },
    @{ M = 'restart';        Show = 'If the client has a Restart headset button: press it, and Enter when the home is back. If it has none: s.'; Max = 120 },
    @{ M = 'client-close';   Show = 'Close the Pimax client fully: window AND tray icon. Then Enter.'; Check = $CheckClosed },
    @{ M = 'closed-led';     Show = 'Wait 30 seconds. Then type the colour of the headset light (for example green, blue, red, off) and Enter.'; Min = 30 },
    @{ M = 'client-start';   Show = 'Start the Pimax client again. Enter when the headset shows the Pimax home.'; Max = 120 }
)
[void](Run-Part 'sleep-wake' 'Part 4 of 7: sleep and wake' 'The USB recording runs while the headset goes to sleep, wakes up, and while the client is closed. About 4 minutes. Enter starts the recording.' $steps)

# ---- part 5: controllers
if (Has 'controllers') {
    $steps = @(
        @{ M = 'both-off';     Show = 'Switch both controllers off. Then Enter.' },
        @{ M = 'left-on';      Show = 'Switch the LEFT controller on. Enter when the client shows it connected.'; Max = 60 },
        @{ M = 'right-on';     Show = 'Switch the RIGHT controller on. Enter when the client shows it connected.'; Max = 60 },
        @{ M = 'battery';      Show = 'Open the client page that shows the controller battery. Then Enter.' },
        @{ M = 'unpair-left';  Show = 'Pairing (s skips it if you do not want to pair again): in the client, unpair or remove the LEFT controller. Enter when it is gone.' },
        @{ M = 'pair-left';    Show = 'Put the LEFT controller in pairing mode, then press Pair for the left hand in the client. Enter when it is connected again.'; Max = 120 },
        @{ M = 'unpair-right'; Show = 'Unpair or remove the RIGHT controller in the client. Enter when it is gone.' },
        @{ M = 'pair-right';   Show = 'Put the RIGHT controller in pairing mode, then press Pair for the right hand. Enter when it is connected again.'; Max = 120 },
        @{ M = 'buttons';      Free = $true; Show = 'Press every button once on each controller, move the sticks, squeeze the triggers and grips. Type a name and Enter after each if you like. Enter alone when done.' },
        @{ M = 'vibration';    Show = 'If the client has a vibration test, run it for each hand. Then Enter (s if there is none).' }
    )
    [void](Run-Part 'controllers' 'Part 5 of 7: controllers' 'The USB recording runs while the controllers connect, pair and are used. About 3 to 5 minutes. Enter starts the recording.' $steps)
} else { $script:Done.Add('Part 5 of 7: controllers : not asked (no Pimax controllers)') }

# ---- part 6: eye tracking
if (Has 'eye') {
    $steps = @(
        @{ M = 'worn';        Show = 'Put the headset on and look straight ahead. After about 15 seconds press Enter.'; Min = 15 },
        @{ M = 'off-desk';    Show = 'Take it off and put it on the desk. After about 15 seconds press Enter.'; Min = 15 },
        @{ M = 'worn-again';  Show = 'Put it on again. After about 15 seconds press Enter.'; Min = 15 },
        @{ M = 'calibration'; Show = 'Start the eye calibration in the client and follow every target to the end. Enter when the client says it is done.'; Min = 20; Max = 180 },
        @{ M = 'look-around'; Show = 'Keep it on and look around for 10 seconds. Then Enter.'; Min = 10 }
    )
    $ec = Run-Part 'eye' 'Part 6 of 7: eye tracking' 'The USB recording runs while you wear the headset, take it off, and do one eye calibration in the client. You will wear the headset: the steps are spoken, press Enter on the keyboard. About 3 minutes. After it, the eye tracking files are copied (read only). Enter starts the recording.' $steps
    if ($ec) { try { Copy-EyeFiles } catch { Say ('Copying the eye files failed: ' + $_.Exception.Message) Yellow } }
} else { $script:Done.Add('Part 6 of 7: eye tracking : not asked (no eye tracking)') }

# ---- part 7
Run-Screens

# ================================================================== the bundle
Show-Big 'Making the bundle' 'Copying the client logs, masking serial numbers in the text files, writing README.txt and PRIVATE.txt, then the zip. This can take a few minutes.' 'Cyan'
try { Copy-ClientLogs } catch { Say ('Copying the logs failed: ' + $_.Exception.Message) Yellow }
Scrub-Dir $invDir
Scrub-Dir (Join-Path $bundle 'pvr')
Scrub-Dir (Join-Path $bundle 'eye-files\setpm-files.txt')   # only this list is masked; the eye files themselves stay exact copies

function Present([string]$rel) { Test-Path (Join-Path $bundle $rel) }
$pcaps = @(Get-ChildItem -LiteralPath $capDir -Filter '*.pcap' -ErrorAction SilentlyContinue)

$r = New-Object System.Collections.Generic.List[string]
$r.Add('PiBridge headset data bundle')
$r.Add('=========================')
$r.Add("Kit version $KitVersion, made " + (Get-Date -Format 'yyyy-MM-dd HH:mm') + ' (local time).')
$r.Add("Headset: $modelName" + $(if ($shownName) { "  (the client calls it: $shownName)" } else { '' }) + $(if ($script:PvrProduct) { "  (the client library says: $($script:PvrProduct))" } else { '' }))
$r.Add('Features you told us: ' + (($F.Keys | ForEach-Object { "$_ = $($F[$_])" }) -join ', '))
$r.Add('')
$r.Add('What was done:')
foreach ($d in $script:Done) { $r.Add('  ' + $d) }
if ($script:Quit) { $r.Add('  (finished early with q)') }
$r.Add('')
$r.Add('What is in this folder:')
$r.Add('  captures\     USB recordings (USBPcap, every root hub, whole packets): <time>-<part>-hub<N>.pcap,')
$r.Add('                and <time>-<part>-markers.txt with the time of every step (begin / done / skip / note).')
$r.Add('  inventory\    software.txt (versions of the Pimax client, Tobii runtime, services, displays),')
$r.Add('                usb-devices.csv/.txt (USB ids and product strings, serials masked), edid-*.bin (headset EDIDs, serial fields cleared).')
if (Present 'pvr') { $r.Add('  pvr\          the display data from the Pimax client library: pimax-distortion.json (lens, FOV, eye poses), pimax-info.json (IPD, properties).') }
if (Present 'eye-files') { $r.Add('  eye-files\    exact copies of the Tobii and Pimax eye tracking files, among them your eye profile calibration.setpm.') }
if (Present 'screens') { $r.Add('  screens\      pictures of the client pages.') }
if (Present 'client-logs') { $r.Add('  client-logs\  the Pimax client logs and settings (text files; files with private names were skipped).') }
$r.Add('  run.log       what the script did.')
$r.Add('  PRIVATE.txt   which files are private. READ IT before you send anything.')
$r.Add('')
$r.Add('Nothing was sent to the headset and nothing was uploaded. To send the bundle: open an issue "Support for <your model>"')
$r.Add('at https://github.com/artscout/PiBridge/issues WITHOUT attaching files, and say that a bundle is ready. We reply with a private way to')
$r.Add('share it. Please do not attach it to a public issue: it holds your headset serial (and your eye profile).')
Set-Content -LiteralPath (Join-Path $bundle 'README.txt') -Value $r -Encoding UTF8

$p = New-Object System.Collections.Generic.List[string]
$p.Add('PRIVATE: what in this bundle is personal')
$p.Add('========================================')
$p.Add('Send this bundle only through the private link we give you. We use it only to find the commands your headset needs.')
$p.Add('We publish only those command bytes (in a device description), never the raw recordings, serials or eye profiles,')
$p.Add('and we delete the bundle when your model is done, or earlier if you ask.')
$p.Add('')
if ($pcaps.Count -gt 0) {
    $p.Add('HEADSET SERIAL NUMBER (cannot be masked inside recordings):')
    foreach ($f in $pcaps) { $p.Add('  captures\' + $f.Name) }
    $p.Add('  The USB recordings carry the serial numbers of the headset, the controllers and the eye tracker as the client reads them.')
    if (@($pcaps | Where-Object { $_.Name -match '-eye-' }).Count -gt 0) { $p.Add('  The eye recording also carries your eye profile (biometric) as the client uploads and downloads it.') }
    if (@($pcaps | Where-Object { $_.Name -match '-cold-start-|-sleep-wake-' }).Count -gt 0 -and (Has 'eye')) { $p.Add('  The client start recordings can carry your eye profile too (the client uploads it to the eye tracker at start).') }
    $p.Add('')
}
if (Present 'eye-files') {
    $p.Add('YOUR EYE PROFILE (biometric data: it describes your eyes) and the headset serial:')
    $p.Add('  eye-files\   (calibration.setpm and the other Tobii files; folder names carry the serial; Tobii logs). Exact copies, not masked.')
    $p.Add('')
}
if (Present 'screens') { $p.Add('MAY SHOW THE SERIAL: screens\ (the device or about page of the client often shows it).'); $p.Add('') }
if (Present 'client-logs') { $p.Add('MAY STILL CONTAIN THE SERIAL OR ACCOUNT DETAILS: client-logs\ (serials and the PC/user name are masked where found; logs can hold other forms).'); $p.Add('') }
$p.Add('MASKED (no serial left that the script knows of): inventory\, pvr\ (serial -> SERIAL-MASKED, PC name -> THIS-PC, user folder -> USER),')
$p.Add('  edid-*.bin (the serial fields of the first EDID block cleared).')
$p.Add(("Serial numbers found and masked in the text files: {0}." -f $script:Serials.Count))
Set-Content -LiteralPath (Join-Path $bundle 'PRIVATE.txt') -Value $p -Encoding UTF8

$zip = $bundle + '.zip'
Say 'Making the zip (can take several minutes for big recordings) ...' Cyan
try {
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    if (Test-Path $zip) { Remove-Item $zip -Force }
    [IO.Compression.ZipFile]::CreateFromDirectory($bundle, $zip, [IO.Compression.CompressionLevel]::Optimal, $true)
    Say ("Zip: {0}  ({1} MB)" -f $zip, ((Get-Item $zip).Length / 1MB).ToString('F0', $Inv)) Green
} catch { Say ('The zip could not be made (' + $_.Exception.Message + '). Send the folder instead.') Yellow; $zip = $null }

Show-Big 'All done. Thank you!' ("Everything is in:`n$bundle`n" + $(if ($zip) { "and in the zip next to it.`n" } else { '' }) +
    "Nothing was sent anywhere. Please read PRIVATE.txt in the folder. To send it: open an issue 'Support for $modelName' at https://github.com/artscout/PiBridge/issues, without attaching files, and say that a bundle is ready. We reply with a private way to share it.") 'Green'
Close-Window
