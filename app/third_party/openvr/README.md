# OpenVR SDK header (vendored)

`openvr.h` and `LICENSE` are copied unchanged from the OpenVR SDK 2.15.6 by Valve Corporation
(BSD 3-Clause, see `LICENSE`; upstream https://github.com/ValveSoftware/openvr, tag v2.15.6, file `headers/openvr.h`).
Only the header is vendored, no binary: the app loads `libopenvr_api.so` from the installed SteamVR at run time
with `dlopen` (`app/src/core/steamvr_openvr.cpp`), so it has no link-time dependency on OpenVR.
