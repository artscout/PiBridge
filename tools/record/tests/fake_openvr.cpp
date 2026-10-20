// SPDX-License-Identifier: MIT
// A fake libopenvr_api.so for pibridge-record-mirror's tests (PIBRIDGE_OPENVR_LIB): the four entry points the mirror
// loads. FAKE_OPENVR_INIT_ERROR (default 121, VRInitError_Init_NoServerForBackgroundApp) is what VR_InitInternal2
// answers; with 0 the connection succeeds but no interface is given out. It never reaches EGL or a GPU.
#include <cstdint>
#include <cstdlib>

extern "C" {

__attribute__((visibility("default"))) uint32_t VR_InitInternal2(int* err, int /*type*/, const char* /*startup*/) {
    const char* e = std::getenv("FAKE_OPENVR_INIT_ERROR");
    *err = e != nullptr ? std::atoi(e) : 121;
    return *err == 0 ? 1u : 0u;
}

__attribute__((visibility("default"))) void VR_ShutdownInternal() {}

__attribute__((visibility("default"))) void* VR_GetGenericInterface(const char* /*version*/, int* err) {
    *err = 105; // VRInitError_Init_InterfaceNotFound
    return nullptr;
}

__attribute__((visibility("default"))) bool VR_IsInterfaceVersionValid(const char* /*version*/) { return true; }

__attribute__((visibility("default"))) const char* VR_GetVRInitErrorAsSymbol(int /*err*/) { return "VRInitError_Fake"; }
}
