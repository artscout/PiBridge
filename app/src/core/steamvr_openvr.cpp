// SPDX-License-Identifier: MIT
// The real SteamVR source: libopenvr_api.so of the installed SteamVR, loaded with dlopen (no link-time dependency),
// used as a vr::VRApplication_Background client. openvr.h's inline VR_Init / VR_Shutdown are replicated with
// function pointers. Only reads: device class, role, model number, controller type, serial number (shown for
// support, never logged or stored), battery, pose, activity, the headset's tracking universe, IPD, display
// frequency and recommended render size, and the compositor's process id (IVRApplications). Every call happens on the worker thread that owns the source.
#include <dlfcn.h>

#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QFileInfo>
#include <array>
#include <openvr.h>

#include "steamvr.h"

namespace pibridge::app {

namespace {

using InitInternal2Fn = uint32_t (*)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
using ShutdownInternalFn = void (*)();
using GetGenericInterfaceFn = void* (*)(const char*, vr::EVRInitError*);
using IsInterfaceVersionValidFn = bool (*)(const char*);
using ErrorSymbolFn = const char* (*)(vr::EVRInitError);

template <typename F>
F sym(void* lib, const char* name) {
    return reinterpret_cast<F>(dlsym(lib, name));
}

QString initErrorWords(vr::EVRInitError e, ErrorSymbolFn symbol) {
    switch (e) {
    case vr::VRInitError_Init_NoServerForBackgroundApp: return QStringLiteral("SteamVR is not ready yet");
    case vr::VRInitError_Init_InterfaceNotFound: return QStringLiteral("this SteamVR is too old for the app");
    case vr::VRInitError_Init_HmdNotFound:
    case vr::VRInitError_Init_HmdNotFoundPresenceFailed: return QStringLiteral("SteamVR sees no headset");
    default: break;
    }
    const char* s = symbol ? symbol(e) : nullptr;
    return QStringLiteral("SteamVR refused the connection (%1)")
        .arg(s ? QString::fromLatin1(s) : QString::number(static_cast<int>(e)));
}

class OpenVrSource : public SteamVrSource {
public:
    ~OpenVrSource() override { disconnect(); }

    VrConnect connect() override {
        if (QString err; !load(err)) {
            return {VrConnect::NotInstalled, err};
        }
        vr::EVRInitError e = vr::VRInitError_None;
        initInternal2_(&e, vr::VRApplication_Background, nullptr); // the token is only for the header's context
        if (e != vr::VRInitError_None) {
            return {VrConnect::Failed, initErrorWords(e, errorSymbol_)};
        }
        live_ = true; // from here on VR_ShutdownInternal must be called
        if (!isInterfaceVersionValid_(vr::IVRSystem_Version)) {
            disconnect();
            return {VrConnect::Failed, initErrorWords(vr::VRInitError_Init_InterfaceNotFound, errorSymbol_)};
        }
        system_ = static_cast<vr::IVRSystem*>(getGenericInterface_(vr::IVRSystem_Version, &e));
        if (!system_ || e != vr::VRInitError_None) {
            disconnect();
            return {VrConnect::Failed, initErrorWords(e != vr::VRInitError_None ? e : vr::VRInitError_Init_InterfaceNotFound,
                                                      errorSymbol_)};
        }
        // As VR_Init does: tell the runtime which SDK the interface layout is from.
        e = system_->SetSDKVersion(vr::k_nSteamVRVersionMajor, vr::k_nSteamVRVersionMinor, vr::k_nSteamVRVersionBuild);
        if (e != vr::VRInitError_None) {
            disconnect();
            return {VrConnect::Failed, initErrorWords(e, errorSymbol_)};
        }
        // IVRApplications through the same generic-interface path, only to ask whether the compositor runs (the
        // Apps page waits for it). Optional: without it the app falls back to a vrcompositor process in /proc.
        apps_ = nullptr;
        if (isInterfaceVersionValid_(vr::IVRApplications_Version)) {
            vr::EVRInitError ae = vr::VRInitError_None;
            auto* a = static_cast<vr::IVRApplications*>(getGenericInterface_(vr::IVRApplications_Version, &ae));
            apps_ = ae == vr::VRInitError_None ? a : nullptr;
        }
        return {VrConnect::Ok, {}};
    }

    std::optional<std::vector<VrDevice>> poll() override {
        if (!system_) {
            return std::nullopt;
        }
        vr::VREvent_t ev{};
        for (int i = 0; i < 512 && system_->PollNextEvent(&ev, sizeof ev); i++) {
            if (ev.eventType == vr::VREvent_Quit) {
                return std::nullopt; // the link calls disconnect()
            }
        }
        std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poses{};
        system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses.data(),
                                                 vr::k_unMaxTrackedDeviceCount);
        std::vector<VrDevice> out;
        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (!system_->IsTrackedDeviceConnected(i)) {
                continue;
            }
            VrDevice d;
            d.index = static_cast<int>(i);
            switch (system_->GetTrackedDeviceClass(i)) {
            case vr::TrackedDeviceClass_HMD: d.cls = VrClass::Hmd; break;
            case vr::TrackedDeviceClass_Controller: d.cls = VrClass::Controller; break;
            case vr::TrackedDeviceClass_TrackingReference: d.cls = VrClass::TrackingReference; break;
            case vr::TrackedDeviceClass_GenericTracker: d.cls = VrClass::GenericTracker; break;
            default: d.cls = VrClass::Other; break;
            }
            switch (system_->GetControllerRoleForTrackedDeviceIndex(i)) {
            case vr::TrackedControllerRole_LeftHand: d.role = VrRole::Left; break;
            case vr::TrackedControllerRole_RightHand: d.role = VrRole::Right; break;
            default: d.role = VrRole::None; break;
            }
            d.model = stringProp(i, vr::Prop_ModelNumber_String);
            d.controllerType = stringProp(i, vr::Prop_ControllerType_String);
            d.serial = stringProp(i, vr::Prop_SerialNumber_String);
            vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
            d.hasBattery = system_->GetBoolTrackedDeviceProperty(i, vr::Prop_DeviceProvidesBatteryStatus_Bool, &pe) &&
                           pe == vr::TrackedProp_Success;
            if (d.hasBattery) {
                d.battery = system_->GetFloatTrackedDeviceProperty(i, vr::Prop_DeviceBatteryPercentage_Float, &pe);
                if (pe != vr::TrackedProp_Success) {
                    d.hasBattery = false;
                }
                d.charging = system_->GetBoolTrackedDeviceProperty(i, vr::Prop_DeviceIsCharging_Bool, &pe) &&
                             pe == vr::TrackedProp_Success;
            }
            d.poseValid = poses[i].bPoseIsValid && poses[i].bDeviceIsConnected;
            d.trackingResult = static_cast<int>(poses[i].eTrackingResult);
            d.activity = static_cast<int>(system_->GetTrackedDeviceActivityLevel(i));
            if (d.cls == VrClass::Hmd) {
                vr::ETrackedPropertyError ue = vr::TrackedProp_Success;
                const uint64_t u = system_->GetUint64TrackedDeviceProperty(i, vr::Prop_CurrentUniverseId_Uint64, &ue);
                d.universe = ue == vr::TrackedProp_Success ? static_cast<quint64>(u) : 0;
                vr::ETrackedPropertyError fe = vr::TrackedProp_Success;
                const float ipd = system_->GetFloatTrackedDeviceProperty(i, vr::Prop_UserIpdMeters_Float, &fe);
                d.ipdMeters = fe == vr::TrackedProp_Success && ipd > 0 ? ipd : 0;
                const float hz = system_->GetFloatTrackedDeviceProperty(i, vr::Prop_DisplayFrequency_Float, &fe);
                d.displayHz = fe == vr::TrackedProp_Success && hz > 0 ? hz : 0;
                uint32_t w = 0, h = 0;
                system_->GetRecommendedRenderTargetSize(&w, &h);
                d.renderWidth = w;
                d.renderHeight = h;
            }
            out.push_back(std::move(d));
        }
        // not written (live 2026-10-19): under driver_monado SteamVR's standing pose equals the raw pose (the head read
        // 2.81 m in standing with a +2.71 m offset), so applying universe 2's -1.52 m stage height raised the world by
        // that much again ("a giant hall"); pibridge-roommap then puts the floor at raw y = 0, which is SteamVR's floor
        // (void)writeStage(out);
        return out;
    }

    // SteamVR's standing frame for pibridge-roommap (steamvr.h steamVrStagePath), written when it changes.
    void writeStage(const std::vector<VrDevice>& devs) {
        quint64 universe = 0;
        for (const VrDevice& d : devs) {
            if (d.cls == VrClass::Hmd) {
                universe = d.universe;
            }
        }
        if (stagePid_ <= 0 || !QFileInfo::exists(QStringLiteral("/proc/%1").arg(stagePid_))) {
            stagePid_ = vrserverPid();
        }
        if (stagePid_ <= 0) {
            return;
        }
        const vr::HmdMatrix34_t m = system_->GetRawZeroPoseToStandingAbsoluteTrackingPose();
        std::array<float, 12> a{};
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 4; c++) {
                a[size_t(r * 4 + c)] = m.m[r][c];
            }
        }
        const QString text = steamVrStageText(stagePid_, universe, a);
        if (text == stageText_) {
            return;
        }
        const QString path = steamVrStagePath();
        QDir().mkpath(QFileInfo(path).absolutePath());
        QSaveFile f(path);
        if (f.open(QIODevice::WriteOnly) && f.write(text.toUtf8()) >= 0 && f.commit()) {
            stageText_ = text;
        }
    }

    int compositorPid() override {
        if (!system_ || !apps_) {
            return -1;
        }
        return static_cast<int>(apps_->GetApplicationProcessId("openvr.component.vrcompositor"));
    }

    void disconnect() override {
        system_ = nullptr;
        apps_ = nullptr;
        if (live_) {
            shutdownInternal_();
            live_ = false;
        }
    }

private:
    // Load the library once; it stays loaded for the life of the app (unloading it while vrclient's threads may
    // still run is not safe).
    bool load(QString& err) {
        if (lib_) {
            return true;
        }
        const QString vrpath = openVrPathsFile();
        const QString runtime = openVrRuntimeDir(vrpath);
        if (runtime.isEmpty()) {
            err = QStringLiteral("no SteamVR runtime in %1").arg(vrpath);
            return false;
        }
        const QString path = openVrApiLibrary(runtime);
        if (!QFileInfo(path).isFile()) {
            err = QStringLiteral("%1 is missing").arg(path);
            return false;
        }
        void* lib = dlopen(QFile::encodeName(path).constData(), RTLD_NOW | RTLD_LOCAL);
        if (!lib) {
            err = QStringLiteral("could not load %1").arg(path);
            return false;
        }
        initInternal2_ = sym<InitInternal2Fn>(lib, "VR_InitInternal2");
        shutdownInternal_ = sym<ShutdownInternalFn>(lib, "VR_ShutdownInternal");
        getGenericInterface_ = sym<GetGenericInterfaceFn>(lib, "VR_GetGenericInterface");
        isInterfaceVersionValid_ = sym<IsInterfaceVersionValidFn>(lib, "VR_IsInterfaceVersionValid");
        errorSymbol_ = sym<ErrorSymbolFn>(lib, "VR_GetVRInitErrorAsSymbol"); // optional, for the error text
        if (!initInternal2_ || !shutdownInternal_ || !getGenericInterface_ || !isInterfaceVersionValid_) {
            dlclose(lib); // nothing of it ran yet
            initInternal2_ = nullptr;
            err = QStringLiteral("%1 lacks the OpenVR functions").arg(path);
            return false;
        }
        lib_ = lib;
        return true;
    }

    QString stringProp(vr::TrackedDeviceIndex_t i, vr::ETrackedDeviceProperty prop) {
        std::array<char, 256> buf{};
        vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
        const uint32_t n = system_->GetStringTrackedDeviceProperty(i, prop, buf.data(), buf.size(), &pe);
        if (pe != vr::TrackedProp_Success || n == 0) {
            return {}; // missing, or longer than any model name we want to show
        }
        buf.back() = '\0';
        return QString::fromUtf8(buf.data());
    }

    void* lib_ = nullptr;
    InitInternal2Fn initInternal2_ = nullptr;
    ShutdownInternalFn shutdownInternal_ = nullptr;
    GetGenericInterfaceFn getGenericInterface_ = nullptr;
    IsInterfaceVersionValidFn isInterfaceVersionValid_ = nullptr;
    ErrorSymbolFn errorSymbol_ = nullptr;
    vr::IVRSystem* system_ = nullptr;
    qint64 stagePid_ = 0;
    QString stageText_;
    vr::IVRApplications* apps_ = nullptr;
    bool live_ = false;
};

} // namespace

std::unique_ptr<SteamVrSource> makeOpenVrSource() {
    return std::make_unique<OpenVrSource>();
}

} // namespace pibridge::app
