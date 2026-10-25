// SPDX-License-Identifier: MIT
// The version string a built Monado carries when it links libpibridge_pimax.a (docs/monado-abi.md, plan M9): a marker
// like omarchyvr's PIMAX_* needles, so `grep -ac "PIBRIDGE_PIMAX_ABI"` on monado-service or driver_monado.so tells a
// PiBridge-linked build from a Rust-linked one, and which C ABI spec (omarchyvr commit, monado-abi/spec.txt) it carries.
// Nothing calls it: "used" keeps the compiler from dropping it and "retain" keeps it through --gc-sections; the
// symbol itself is local after the archive's objcopy (only pimax_* stay global).

#ifndef PIBRIDGE_PIMAX_ABI_SPEC
#error "PIBRIDGE_PIMAX_ABI_SPEC (the pinned omarchyvr commit) comes from monado-abi/CMakeLists.txt"
#endif

// 1 = the C ABI of the seven headers in include/ (pimax.h, pimax_hands.h, pimax_ctrlfuse.h, pimax_inhand.h,
// pimax_eye.h, pimax_cam.h, pimax_lens_rs.h). Raise it when PiBridge changes that ABI.
#define PIBRIDGE_PIMAX_ABI_VERSION "1"

extern "C" __attribute__((used, retain, visibility("hidden"))) const char pibridge_pimax_abi_id[] =
    "PIBRIDGE_PIMAX_ABI " PIBRIDGE_PIMAX_ABI_VERSION " spec omarchyvr " PIBRIDGE_PIMAX_ABI_SPEC;
