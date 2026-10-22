# Help us support your headset

PiBridge runs one Pimax headset today: the **Crystal Super micro-OLED**. Other Pimax headsets speak the same family of
USB commands, but each model differs in the details: other product ids, other wake and keepalive sequences, other
display modes, other settings, a Lighthouse switch or not, an eye tracker or not. We only ever send a headset the
bytes the official Pimax client sends it (see [CONVENTIONS.md](CONVENTIONS.md), Safety), so to add a model we need a
recording of what the Pimax client does with that model.

If you own another Pimax headset and a Windows PC with the Pimax client, you can make that recording in about
15 minutes with one guided script: the **headset data kit** in [`windows/`](../windows/).

## What the kit does

- It runs on Windows, next to the official Pimax client (Pimax Play / Pimax EVO).
- It **listens** to the USB traffic with [USBPcap](https://desowin.org/usbpcap/) while you use the client:
  start it, change settings, put the headset to sleep and wake it, pair the controllers, calibrate the eyes.
- It **reads** version numbers, the USB device list, the headset's display data (lens, field of view, IPD; through
  the Pimax client's own library, "get" calls only) and, if you have eye tracking, the eye tracking files.
- It **never sends anything** to the headset, the controllers or the eye tracker, never changes a Pimax setting
  (you change them yourself, in the client), never installs anything, and **never uploads anything**. Everything
  stays in one folder on your PC until you decide to send it.

## What to prepare

- Windows with the Pimax client installed and working with your headset.
- [USBPcap](https://desowin.org/usbpcap/) installed (the official installer, or tick "Install USBPcap" in the
  Wireshark installer), then restart Windows once. The kit checks for it and stops if it is missing.
- The headset connected; the controllers charged (if you have Pimax controllers).
- SteamVR and games closed.
- About 15 minutes, and up to 10 GB of free disk space (the recordings are big; the zip is smaller).

## How to run it

1. Copy the whole `windows/` folder to the Windows PC (for example to the Desktop or a USB stick with space).
2. Double-click **`PIBRIDGE-HEADSET-DATA.cmd`**. Windows asks once whether to allow changes: answer Yes (USBPcap needs
   Administrator).
3. Tell the script which headset you have, and answer a few yes/no questions (eye tracking? Pimax controllers?
   Lighthouse switch? IPD adjustment?). "Not sure" is fine: you can skip any step later.
4. Follow the steps. Each step is shown in big text and spoken, you do it, then press **Enter**. At every prompt:

   | key | does |
   |---|---|
   | Enter | done, next step |
   | `s` | skip this step (or this part) |
   | `q` | finish now: the bundle is made from what was recorded so far |

   Times shown are hints ("at least 20 seconds"); you decide when a step is done. The time of every Enter is saved.

The parts, in order:

| part | what you do | recorded | about |
|---|---|---|---|
| 1. What is installed | start the client, wait for the Pimax home | versions, USB devices, display data (lens, FOV, eye poses), the client's settings (FOV level, IPD and its range, lens offsets; read only), no USB recording | 1 min |
| 2. Start of the client | close the client fully, wait, start it again | how the client wakes the headset | 2 min |
| 3. Settings | each refresh rate, brightness low/high, tracking mode inside-out / Lighthouse (if the client can switch it), IPD: smallest, largest and a few values in between, typing what the client shows, the IPD buttons on the headset (if it has them) and the automatic IPD (if the client has it), sound, the headset's volume buttons, other settings | the command for each setting | 3-5 min |
| 4. Sleep and wake | standby and back, "Restart headset" if the client has it, the client closed (you tell us the LED colour) | standby, wake, restart and idle | 4 min |
| 5. Controllers | on, battery page, unpair and pair each hand, buttons, vibration | pairing and controller traffic | 3-5 min |
| 6. Eye tracking | wear, take off, wear again, one full eye calibration in the client | the eye tracker's traffic, then a copy of the eye tracking files | 3 min |
| 7. Pictures | open each client page, type a name | a picture of the client window per page | 2 min |

Parts 5 and 6 are only offered if you said yes (or "not sure") to Pimax controllers and eye tracking.

## What you get

One folder `pibridge-headset-data-<model>-<date>-<time>\` next to the script, and the same as a `.zip`:

| inside | what |
|---|---|
| `captures\` | the USB recordings (`*.pcap`, one per USB root hub and part) and a `*-markers.txt` per part with the time of every step |
| `inventory\` | versions of the Pimax client and Tobii runtime, services, displays; the USB device list (serials masked); the headset's EDID (serial fields cleared) |
| `pvr\` | the display data from the Pimax client library (lens distortion, field of view, eye poses, IPD), serial masked |
| `eye-files\` | the Tobii and Pimax eye tracking files, among them your eye profile `calibration.setpm` |
| `screens\` | the pictures of the client pages |
| `client-logs\` | the Pimax client's logs and settings files (text only) |
| `README.txt` | what was done and skipped, and what is in the folder |
| `PRIVATE.txt` | which files are private (read it before sending) |
| `run.log` | what the script did |

## What is private

`PRIVATE.txt` in the folder lists it for your bundle. In short:

- **The headset serial number** is inside the USB recordings (the client reads it from the headset; it cannot be
  masked inside a recording), in the eye tracking folder names, possibly on a client picture (the device page) and
  in the client logs. In the files the kit writes itself (the inventory, the display data) it is replaced by
  `SERIAL-MASKED`, the PC name by `THIS-PC`, the Windows user folder by `USER`.
- **Your eye profile** (`eye-files\...\calibration.setpm`, and its upload/download in the eye recording and possibly
  in the start recordings) is biometric data: it describes your eyes.
- Files whose names look like credentials (tokens, cookies, logins, accounts) are never copied; program files
  (`.exe`, `.dll`, `.sys`) are never copied.

## How to send it

Please **do not** attach the bundle to a public GitHub issue.

1. Open an issue at [github.com/artscout/PiBridge/issues](https://github.com/artscout/PiBridge/issues) titled **"Support for &lt;your model&gt;"**, without any files. Say which
   model and Pimax client version you have, and that a bundle is ready (the size of the zip helps).
2. The maintainer replies with a private way to share it (a private upload link).
3. Upload the zip there.

What we do with it: we use it only to find the commands your model needs. We publish only the command bytes we
derive (in a device description, `devices/<model>.toml`), never the raw recordings, serials or eye profiles, and we
delete the bundle when the model is done, or earlier if you ask.

## What happens next

1. We read the recordings and write `devices/<model>.toml`: the USB ids, and the allowlist of the exact reports the
   Pimax client sent (wake, keepalive, panel, refresh rates, brightness, IPD, sound, tracking mode, controllers).
2. We test it with you: you run a PiBridge build on Linux and tell us what works. We may ask for one more short
   recording (for example a single setting).
3. The model is added to PiBridge and to the table below.

## Models

| model | what we have | status |
|---|---|---|
| Crystal Super micro-OLED | captures and a device description (`devices/crystal-super.toml`), tested with the Lighthouse faceplate | **supported** (alpha) |
| Crystal Super QLED 57 PPD / 50 PPD, Crystal Super UW | the micro-OLED's description may fit, not tested | data wanted |
| Crystal (original, software-switched Lighthouse mode) | nothing yet | data wanted |
| Crystal Light | description from omarchyvr (no captures, USB id not verified), untested | data wanted |
| Dream Air | description from omarchyvr (`devices/dream-air.toml`, from Pimax Play captures; works with omarchyvr's Monado stack), untested in PiBridge | description from omarchyvr, untested |
| Dream Air SE | description from omarchyvr (no captures, USB id not verified), untested | data wanted |
| 8KX | description from omarchyvr (no captures, USB id not verified), untested | data wanted |
| 5K family (5K Plus, 5K XR, 5K Super), 8K / 8K Plus, others | nothing yet | data wanted |

## For maintainers

- The kit: `windows/PIBRIDGE-HEADSET-DATA.cmd` (the only entry point; elevates once) runs `windows/pibridge-headset-data.ps1`,
  which runs `windows/kit/pimax-pvr-dump.ps1 -Parts 1,3` (omarchyvr's PVR dump, read only) for the display data.
  Windows PowerShell 5.1, no modules, CRLF line endings.
- Recordings: USBPcap on every root hub, `-A -s 65535` (whole packets; camera URBs are still cut at about 64 KB by
  USBPcap). File names `<stamp>-<part>-hub<N>.pcap`, the same as omarchyvr's takes; markers
  `<stamp>-<part>-markers.txt`: a `start <local ISO> utc <UTC ISO>` line, then `<seconds> s  begin|done|skip|note|check|quit <step>  <local ISO>`.
- Parts: `cold-start`, `settings`, `sleep-wake`, `controllers`, `eye` (the step names are in the script).
- Tools that read the takes: omarchyvr `analysis/tools/` and `pimax-eye frames` / `pimax-eye profile` (eye start
  list and profile), PiBridge's `pibridge-eye` (`pcap`, `plan`).
- Raw bundles never enter the repo (CONVENTIONS.md: no serials, no private lab data).
