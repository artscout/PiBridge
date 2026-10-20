// SPDX-License-Identifier: MIT
// The OpenVR + EGL backend of pibridge-record-mirror: libopenvr_api.so of the installed SteamVR loaded with dlopen
// (openvr.h's inline VR_Init replicated with function pointers, as the app's steamvr_openvr.cpp does), a
// VRApplication_Background client (it never starts SteamVR and never keeps it running; an Overlay client only when
// SteamVR answers IsNotSceneApplication to a background one), IVRCompositor::GetMirrorTextureGL per eye, and a
// headless EGL context (no window, no surface). Each frame: LockGLSharedTextureForAccess, one scaling blit per eye
// into our own framebuffer (side by side, rows flipped to top-first), Unlock, then glReadPixels into one of three
// pixel buffers with a fence: nothing ever waits for the GPU or for SteamVR. Every GL entry point comes from
// eglGetProcAddress (no libGL link).
#include "pibridge/record/mirror.h"

#include "pibridge_env.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <tuple>
#include <vector>
#include <cstring>
#include <deque>
#include <format>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glcorearb.h>
#include <dlfcn.h>
#include <openvr.h>

namespace pibridge::record {

namespace {

using InitInternal2Fn = uint32_t (*)(vr::EVRInitError*, vr::EVRApplicationType, const char*);
using ShutdownInternalFn = void (*)();
using GetGenericInterfaceFn = void* (*)(const char*, vr::EVRInitError*);
using IsInterfaceVersionValidFn = bool (*)(const char*);
using ErrorDescFn = const char* (*)(vr::EVRInitError);

std::string initErrorWords(vr::EVRInitError e, ErrorDescFn desc) {
    switch (e) {
    case vr::VRInitError_Init_NoServerForBackgroundApp: return "SteamVR is not running";
    case vr::VRInitError_Init_InterfaceNotFound: return "this SteamVR is too old for pibridge-record";
    case vr::VRInitError_Init_HmdNotFound:
    case vr::VRInitError_Init_HmdNotFoundPresenceFailed: return "SteamVR sees no headset";
    default: break;
    }
    return std::format("SteamVR refused the connection ({})", desc ? desc(e) : std::to_string(int(e)).c_str());
}

struct Gl {
    PFNGLGETERRORPROC GetError = nullptr;
    PFNGLBINDTEXTUREPROC BindTexture = nullptr;
    PFNGLGETTEXLEVELPARAMETERIVPROC GetTexLevelParameteriv = nullptr;
    PFNGLGENFRAMEBUFFERSPROC GenFramebuffers = nullptr;
    PFNGLDELETEFRAMEBUFFERSPROC DeleteFramebuffers = nullptr;
    PFNGLBINDFRAMEBUFFERPROC BindFramebuffer = nullptr;
    PFNGLFRAMEBUFFERTEXTURE2DPROC FramebufferTexture2D = nullptr;
    PFNGLFRAMEBUFFERRENDERBUFFERPROC FramebufferRenderbuffer = nullptr;
    PFNGLCHECKFRAMEBUFFERSTATUSPROC CheckFramebufferStatus = nullptr;
    PFNGLGENRENDERBUFFERSPROC GenRenderbuffers = nullptr;
    PFNGLDELETERENDERBUFFERSPROC DeleteRenderbuffers = nullptr;
    PFNGLBINDRENDERBUFFERPROC BindRenderbuffer = nullptr;
    PFNGLRENDERBUFFERSTORAGEPROC RenderbufferStorage = nullptr;
    PFNGLBLITFRAMEBUFFERPROC BlitFramebuffer = nullptr;
    PFNGLGENBUFFERSPROC GenBuffers = nullptr;
    PFNGLDELETEBUFFERSPROC DeleteBuffers = nullptr;
    PFNGLBINDBUFFERPROC BindBuffer = nullptr;
    PFNGLBUFFERDATAPROC BufferData = nullptr;
    PFNGLMAPBUFFERRANGEPROC MapBufferRange = nullptr;
    PFNGLUNMAPBUFFERPROC UnmapBuffer = nullptr;
    PFNGLREADPIXELSPROC ReadPixels = nullptr;
    PFNGLPIXELSTOREIPROC PixelStorei = nullptr;
    PFNGLFENCESYNCPROC FenceSync = nullptr;
    PFNGLCLIENTWAITSYNCPROC ClientWaitSync = nullptr;
    PFNGLDELETESYNCPROC DeleteSync = nullptr;
    PFNGLFLUSHPROC Flush = nullptr;
    PFNGLISTEXTUREPROC IsTexture = nullptr;
    PFNGLGETSTRINGPROC GetString = nullptr;

    template <typename F>
    bool get(F& f, const char* name) {
        f = reinterpret_cast<F>(eglGetProcAddress(name));
        return f != nullptr;
    }
    bool load() {
        return get(GetError, "glGetError") && get(BindTexture, "glBindTexture") &&
               get(GetTexLevelParameteriv, "glGetTexLevelParameteriv") && get(GenFramebuffers, "glGenFramebuffers") &&
               get(DeleteFramebuffers, "glDeleteFramebuffers") && get(BindFramebuffer, "glBindFramebuffer") &&
               get(FramebufferTexture2D, "glFramebufferTexture2D") &&
               get(FramebufferRenderbuffer, "glFramebufferRenderbuffer") &&
               get(CheckFramebufferStatus, "glCheckFramebufferStatus") && get(GenRenderbuffers, "glGenRenderbuffers") &&
               get(DeleteRenderbuffers, "glDeleteRenderbuffers") && get(BindRenderbuffer, "glBindRenderbuffer") &&
               get(RenderbufferStorage, "glRenderbufferStorage") && get(BlitFramebuffer, "glBlitFramebuffer") &&
               get(GenBuffers, "glGenBuffers") && get(DeleteBuffers, "glDeleteBuffers") && get(BindBuffer, "glBindBuffer") &&
               get(BufferData, "glBufferData") && get(MapBufferRange, "glMapBufferRange") &&
               get(UnmapBuffer, "glUnmapBuffer") && get(ReadPixels, "glReadPixels") && get(PixelStorei, "glPixelStorei") &&
               get(FenceSync, "glFenceSync") && get(ClientWaitSync, "glClientWaitSync") && get(DeleteSync, "glDeleteSync") &&
               get(Flush, "glFlush") && get(IsTexture, "glIsTexture") && get(GetString, "glGetString");
    }
};

class OpenVrMirror final : public MirrorBackend {
public:
    explicit OpenVrMirror(std::filesystem::path lib) : libPath_(std::move(lib)) {
        const char* f = pibridge::env("RECORD_MIRROR_FLIP");
        flip_ = f == nullptr || std::strcmp(f, "0") != 0;
    }
    ~OpenVrMirror() override { shutdown(); }

    std::string init(std::uint32_t eyes) override {
        eyes_ = eyes;
        if (std::string e = load(); !e.empty()) {
            return e;
        }
        if (std::string e = connect(vr::VRApplication_Background); !e.empty()) {
            return e;
        }
        if (std::string e = makeContext(); !e.empty()) {
            return e;
        }
        int err = mirrorTextures();
        if (err == vr::VRCompositorError_IsNotSceneApplication) {
            // some SteamVR versions share the mirror with overlay clients only; SteamVR runs (checked above), so
            // an overlay client does not start it
            releaseTextures();
            disconnect();
            if (std::string e = connect(vr::VRApplication_Overlay); !e.empty()) {
                return e;
            }
            type_ = "overlay";
            err = mirrorTextures();
        }
        if (err != vr::VRCompositorError_None) {
            return "SteamVR's mirror texture is not available: " + compositorErrorWords(err);
        }
        return {};
    }

    std::pair<std::uint32_t, std::uint32_t> eyeSize() override {
        const auto query = [&](GLenum target) {
            GLint w = 0, h = 0;
            while (gl_.GetError() != GL_NO_ERROR) {
            }
            gl_.BindTexture(target, eye_[0].tex);
            gl_.GetTexLevelParameteriv(target, 0, GL_TEXTURE_WIDTH, &w);
            gl_.GetTexLevelParameteriv(target, 0, GL_TEXTURE_HEIGHT, &h);
            const GLenum err = gl_.GetError();
            gl_.BindTexture(target, 0);
            return std::tuple{w, h, err};
        };
        comp_->LockGLSharedTextureForAccess(eye_[0].handle);
        const bool isTex = gl_.IsTexture(eye_[0].tex) == GL_TRUE;
        const auto [w2, h2, e2] = query(GL_TEXTURE_2D);
        auto [wr, hr, er] = std::tuple{0, 0, GLenum(GL_NO_ERROR)};
        if (w2 <= 0 || h2 <= 0) {
            std::tie(wr, hr, er) = query(GL_TEXTURE_RECTANGLE);
        }
        comp_->UnlockGLSharedTextureForAccess(eye_[0].handle);
        std::uint32_t rw = 0, rh = 0;
        if (sys_ != nullptr) {
            sys_->GetRecommendedRenderTargetSize(&rw, &rh);
        }
        std::string how;
        const auto size = chooseMirrorSize({w2, h2}, {wr, hr}, {rw, rh}, how);
        // a texture name the context does not know: the size of SteamVR's own render target would only hide that
        const auto chosen = isTex ? size : std::pair<std::uint32_t, std::uint32_t>{0, 0};
        srcW_ = chosen.first;
        srcH_ = chosen.second;
        target_ = (w2 > 0 && h2 > 0) || !(wr > 0 && hr > 0) ? GL_TEXTURE_2D : GL_TEXTURE_RECTANGLE;
        std::fprintf(stderr,
                     "pibridge-record-mirror: mirror texture %u (handle %p): %s; 2D %dx%d (GL error %#x), rectangle %dx%d "
                     "(%#x), recommended %ux%u -> %ux%u from %s\n",
                     eye_[0].tex, eye_[0].handle, isTex ? "a texture in this context" : "NOT a texture in this context",
                     w2, h2, e2, wr, hr, er, rw, rh, srcW_, srcH_, isTex ? how.c_str() : "none");
        if (srcW_ == 0 || srcH_ == 0) {
            sizeProblem_ = std::format("texture {} is {} in the OpenGL context on {} ({}); 2D {}x{}, rectangle {}x{}; "
                                       "see record-mirror.log",
                                       eye_[0].tex, isTex ? "empty" : "unknown", device_, renderer_, w2, h2, wr, hr);
        }
        return {srcW_, srcH_};
    }

    std::string sizeProblem() const override { return sizeProblem_; }

    std::string configure(std::uint32_t eyes, std::uint32_t eyeW, std::uint32_t eyeH) override {
        eyeW_ = eyeW;
        eyeH_ = eyeH;
        outW_ = eyeW * eyes;
        gl_.GenRenderbuffers(1, &rbo_);
        gl_.BindRenderbuffer(GL_RENDERBUFFER, rbo_);
        gl_.RenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, GLsizei(outW_), GLsizei(eyeH_));
        gl_.GenFramebuffers(1, &drawFbo_);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, drawFbo_);
        gl_.FramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo_);
        if (gl_.CheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            return "OpenGL could not make the recording framebuffer";
        }
        gl_.GenFramebuffers(1, &readFbo_);
        for (auto& p : pbo_) {
            gl_.GenBuffers(1, &p.buf);
            gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, p.buf);
            gl_.BufferData(GL_PIXEL_PACK_BUFFER, GLsizeiptr(std::size_t(outW_) * eyeH_ * 4), nullptr, GL_STREAM_READ);
        }
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        if (const GLenum e = gl_.GetError(); e != GL_NO_ERROR) {
            return std::format("OpenGL error {:#x} while setting up the readback", e);
        }
        return {};
    }

    Submit submit(std::uint64_t seq, std::uint64_t ts, std::string& error) override {
        Pbo* free = nullptr;
        for (auto& p : pbo_) {
            if (p.fence == nullptr) {
                free = &p;
                break;
            }
        }
        if (free == nullptr) {
            return Submit::Busy;
        }
        gl_.BindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFbo_);
        for (std::uint32_t i = 0; i < eyes_; i++) {
            comp_->LockGLSharedTextureForAccess(eye_[i].handle);
            gl_.BindFramebuffer(GL_READ_FRAMEBUFFER, readFbo_);
            gl_.FramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, target_, eye_[i].tex, 0);
            const GLint x0 = GLint(i * eyeW_), x1 = GLint((i + 1) * eyeW_);
            // GL rows go bottom-up: the flipped blit makes glReadPixels return the top row first
            gl_.BlitFramebuffer(0, 0, GLint(srcW_), GLint(srcH_), x0, flip_ ? GLint(eyeH_) : 0, x1, flip_ ? 0 : GLint(eyeH_),
                                GL_COLOR_BUFFER_BIT, GL_LINEAR);
            gl_.FramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, target_, 0, 0);
            comp_->UnlockGLSharedTextureForAccess(eye_[i].handle);
        }
        gl_.BindFramebuffer(GL_READ_FRAMEBUFFER, drawFbo_);
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, free->buf);
        gl_.PixelStorei(GL_PACK_ALIGNMENT, 4);
        gl_.ReadPixels(0, 0, GLsizei(outW_), GLsizei(eyeH_), GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        free->fence = gl_.FenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        gl_.Flush();
        if (const GLenum e = gl_.GetError(); e != GL_NO_ERROR) {
            error = std::format("OpenGL error {:#x} while reading SteamVR's mirror texture", e);
            return Submit::Failed;
        }
        free->seq = seq;
        free->ts = ts;
        pending_.push_back(free);
        return Submit::Queued;
    }

    std::optional<MirrorFrame> poll() override {
        if (pending_.empty() || mapped_ != nullptr) {
            return std::nullopt;
        }
        Pbo* p = pending_.front();
        const GLenum r = gl_.ClientWaitSync(p->fence, 0, 0);
        if (r != GL_ALREADY_SIGNALED && r != GL_CONDITION_SATISFIED) {
            return std::nullopt;
        }
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, p->buf);
        const std::size_t n = std::size_t(outW_) * eyeH_ * 4;
        const void* data = gl_.MapBufferRange(GL_PIXEL_PACK_BUFFER, 0, GLsizeiptr(n), GL_MAP_READ_BIT);
        if (data == nullptr) {
            gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            freePbo(p);
            return std::nullopt;
        }
        mapped_ = p;
        return MirrorFrame{p->seq, p->ts, {static_cast<const std::uint8_t*>(data), n}};
    }

    void release() override {
        if (mapped_ == nullptr) {
            return;
        }
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, mapped_->buf);
        gl_.UnmapBuffer(GL_PIXEL_PACK_BUFFER);
        gl_.BindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        freePbo(mapped_);
        mapped_ = nullptr;
    }

    bool quitRequested() override {
        vr::VREvent_t ev{};
        while (sys_ != nullptr && sys_->PollNextEvent(&ev, sizeof ev)) {
            if (ev.eventType == vr::VREvent_Quit) {
                sys_->AcknowledgeQuit_Exiting();
                quit_ = true;
            }
        }
        return quit_;
    }

    std::string note() const override {
        return std::format("SteamVR mirror ({} client, {}x{})", type_, srcW_, srcH_);
    }
    std::string device() const { return device_;
    }

private:
    struct Pbo {
        GLuint buf = 0;
        GLsync fence = nullptr;
        std::uint64_t seq = 0, ts = 0;
    };
    struct Eye {
        vr::glUInt_t tex = 0;
        vr::glSharedTextureHandle_t handle = nullptr;
    };

    void freePbo(Pbo* p) {
        if (p->fence != nullptr) {
            gl_.DeleteSync(p->fence);
            p->fence = nullptr;
        }
        if (!pending_.empty() && pending_.front() == p) {
            pending_.pop_front();
        }
    }

    std::string load() {
        if (libPath_.empty()) {
            return "SteamVR's OpenVR library (libopenvr_api.so) was not found: is SteamVR installed?";
        }
        lib_ = dlopen(libPath_.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (lib_ == nullptr) {
            return "could not load " + libPath_.string();
        }
        init_ = reinterpret_cast<InitInternal2Fn>(dlsym(lib_, "VR_InitInternal2"));
        shutdown_ = reinterpret_cast<ShutdownInternalFn>(dlsym(lib_, "VR_ShutdownInternal"));
        generic_ = reinterpret_cast<GetGenericInterfaceFn>(dlsym(lib_, "VR_GetGenericInterface"));
        valid_ = reinterpret_cast<IsInterfaceVersionValidFn>(dlsym(lib_, "VR_IsInterfaceVersionValid"));
        desc_ = reinterpret_cast<ErrorDescFn>(dlsym(lib_, "VR_GetVRInitErrorAsSymbol"));
        if (!init_ || !shutdown_ || !generic_ || !valid_) {
            return libPath_.string() + " lacks the OpenVR functions";
        }
        return {};
    }

    std::string connect(vr::EVRApplicationType type) {
        vr::EVRInitError err = vr::VRInitError_None;
        init_(&err, type, nullptr);
        if (err != vr::VRInitError_None) {
            return initErrorWords(err, desc_);
        }
        live_ = true;
        if (!valid_(vr::IVRSystem_Version) || !valid_(vr::IVRCompositor_Version)) {
            return "this SteamVR is too old for pibridge-record (no " + std::string(vr::IVRCompositor_Version) + ")";
        }
        sys_ = static_cast<vr::IVRSystem*>(generic_(vr::IVRSystem_Version, &err));
        comp_ = static_cast<vr::IVRCompositor*>(generic_(vr::IVRCompositor_Version, &err));
        if (sys_ == nullptr || comp_ == nullptr) {
            return "SteamVR gives no compositor interface to this client";
        }
        return {};
    }

    void disconnect() {
        sys_ = nullptr;
        comp_ = nullptr;
        if (live_) {
            shutdown_();
            live_ = false;
        }
    }

    // Make a context on `dpy` (already initialised): desktop OpenGL 3.3, surfaceless or a 1x1 pbuffer.
    std::string contextOn(EGLDisplay dpy) {
        if (!eglBindAPI(EGL_OPENGL_API)) {
            return "EGL has no desktop OpenGL";
        }
        const EGLint cfgAttr[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RED_SIZE, 8,
                                  EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE};
        EGLConfig cfg = nullptr;
        EGLint nc = 0;
        if (!eglChooseConfig(dpy, cfgAttr, &cfg, 1, &nc) || nc < 1) {
            return "EGL has no OpenGL configuration";
        }
        const EGLint ctxAttr[] = {EGL_CONTEXT_MAJOR_VERSION, 3, EGL_CONTEXT_MINOR_VERSION, 3, EGL_NONE};
        EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctxAttr);
        if (ctx == EGL_NO_CONTEXT) {
            return "EGL could not create an OpenGL 3.3 context";
        }
        EGLSurface surface = EGL_NO_SURFACE;
        if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
            const EGLint pb[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
            surface = eglCreatePbufferSurface(dpy, cfg, pb);
            if (surface == EGL_NO_SURFACE || !eglMakeCurrent(dpy, surface, surface, ctx)) {
                if (surface != EGL_NO_SURFACE) {
                    eglDestroySurface(dpy, surface);
                }
                eglDestroyContext(dpy, ctx);
                return "EGL could not make the OpenGL context current";
            }
        }
        dpy_ = dpy;
        ctx_ = ctx;
        surface_ = surface;
        return {};
    }

    // SteamVR's GL mirror is shared with the NVIDIA driver's OpenGL: an EGL device of NVIDIA's first (GLVND's
    // default display can be Mesa's, which on an NVIDIA card fails with "failed to create dri2 screen" and gives a
    // context in which the mirror's texture name means nothing: size 0). PIBRIDGE_RECORD_EGL_DEVICE=N forces device N,
    // PIBRIDGE_RECORD_EGL_DEVICE=default the default display.
    std::string makeContext() {
        const char* forcedEnv = pibridge::env("RECORD_EGL_DEVICE");
        const bool useDefault = forcedEnv != nullptr && std::strcmp(forcedEnv, "default") == 0;
        auto queryDevices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
        auto platformDisplay =
            reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        std::string lastErr = "no EGL device";
        if (!useDefault && queryDevices && platformDisplay) {
            std::array<EGLDeviceEXT, 16> devs{};
            EGLint n = 0;
            if (queryDevices(EGLint(devs.size()), devs.data(), &n) && n > 0) {
                std::vector<std::string> vendors(static_cast<std::size_t>(n));
                std::vector<EGLDisplay> dpys(static_cast<std::size_t>(n), EGL_NO_DISPLAY);
                for (EGLint i = 0; i < n; i++) {
                    EGLDisplay d = platformDisplay(EGL_PLATFORM_DEVICE_EXT, devs[std::size_t(i)], nullptr);
                    EGLint ma = 0, mi = 0;
                    if (d != EGL_NO_DISPLAY && eglInitialize(d, &ma, &mi)) {
                        const char* v = eglQueryString(d, EGL_VENDOR);
                        vendors[std::size_t(i)] = v ? v : "?";
                        dpys[std::size_t(i)] = d;
                    }
                    std::fprintf(stderr, "pibridge-record-mirror: EGL device %d: %s\n", int(i),
                                 vendors[std::size_t(i)].empty() ? "(does not start)" : vendors[std::size_t(i)].c_str());
                }
                const int forced = forcedEnv != nullptr ? std::atoi(forcedEnv) : -1;
                for (const int i : eglDeviceOrder(vendors, forced)) {
                    lastErr = contextOn(dpys[std::size_t(i)]);
                    if (lastErr.empty()) {
                        device_ = std::format("EGL device {} ({})", i, vendors[std::size_t(i)]);
                        break;
                    }
                }
                for (EGLint i = 0; i < n; i++) {
                    if (dpys[std::size_t(i)] != EGL_NO_DISPLAY && dpys[std::size_t(i)] != dpy_) {
                        eglTerminate(dpys[std::size_t(i)]);
                    }
                }
            }
        }
        if (ctx_ == EGL_NO_CONTEXT) {
            EGLDisplay d = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            EGLint ma = 0, mi = 0;
            if (d == EGL_NO_DISPLAY || !eglInitialize(d, &ma, &mi)) {
                return "no EGL display for the headless OpenGL context (" + lastErr + ")";
            }
            if (std::string e = contextOn(d); !e.empty()) {
                return e;
            }
            const char* v = eglQueryString(d, EGL_VENDOR);
            device_ = std::format("the default EGL display ({})", v ? v : "?");
        }
        if (!gl_.load()) {
            return "OpenGL lacks the functions pibridge-record needs (3.3)";
        }
        const auto str = [&](GLenum e) {
            const auto* p = gl_.GetString ? gl_.GetString(e) : nullptr;
            return p ? std::string(reinterpret_cast<const char*>(p)) : std::string("?");
        };
        renderer_ = str(GL_VENDOR) + " " + str(GL_RENDERER);
        std::fprintf(stderr, "pibridge-record-mirror: OpenGL on %s: %s, %s\n", device_.c_str(), renderer_.c_str(),
                     str(GL_VERSION).c_str());
        return {};
    }

    int mirrorTextures() {
        for (std::uint32_t i = 0; i < eyes_; i++) {
            const auto e = comp_->GetMirrorTextureGL(i == 0 ? vr::Eye_Left : vr::Eye_Right, &eye_[i].tex, &eye_[i].handle);
            std::fprintf(stderr, "pibridge-record-mirror: GetMirrorTextureGL(%s) = %d, texture %u, handle %p (%s client)\n",
                         i == 0 ? "left" : "right", int(e), eye_[i].tex, eye_[i].handle, type_.c_str());
            if (e != vr::VRCompositorError_None) {
                return e;
            }
        }
        return vr::VRCompositorError_None;
    }

    void releaseTextures() {
        for (auto& e : eye_) {
            if (comp_ != nullptr && e.handle != nullptr) {
                comp_->ReleaseSharedGLTexture(e.tex, e.handle);
            }
            e = {};
        }
    }

    void shutdown() {
        if (ctx_ != EGL_NO_CONTEXT) {
            release();
            for (auto& p : pbo_) {
                if (p.fence) {
                    gl_.DeleteSync(p.fence);
                }
                if (p.buf) {
                    gl_.DeleteBuffers(1, &p.buf);
                }
                p = {};
            }
            if (drawFbo_) gl_.DeleteFramebuffers(1, &drawFbo_);
            if (readFbo_) gl_.DeleteFramebuffers(1, &readFbo_);
            if (rbo_) gl_.DeleteRenderbuffers(1, &rbo_);
        }
        releaseTextures();
        if (dpy_ != EGL_NO_DISPLAY) {
            eglMakeCurrent(dpy_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (surface_ != EGL_NO_SURFACE) eglDestroySurface(dpy_, surface_);
            if (ctx_ != EGL_NO_CONTEXT) eglDestroyContext(dpy_, ctx_);
            eglTerminate(dpy_);
            dpy_ = EGL_NO_DISPLAY;
            ctx_ = EGL_NO_CONTEXT;
        }
        disconnect();
        // the library stays loaded (vrclient's threads may still run)
    }

    std::filesystem::path libPath_;
    void* lib_ = nullptr;
    InitInternal2Fn init_ = nullptr;
    ShutdownInternalFn shutdown_ = nullptr;
    GetGenericInterfaceFn generic_ = nullptr;
    IsInterfaceVersionValidFn valid_ = nullptr;
    ErrorDescFn desc_ = nullptr;
    bool live_ = false, quit_ = false, flip_ = true;
    std::string type_ = "background";
    std::string device_, renderer_, sizeProblem_;
    GLenum target_ = GL_TEXTURE_2D;
    vr::IVRSystem* sys_ = nullptr;
    vr::IVRCompositor* comp_ = nullptr;
    EGLDisplay dpy_ = EGL_NO_DISPLAY;
    EGLContext ctx_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    Gl gl_;
    std::uint32_t eyes_ = 1, srcW_ = 0, srcH_ = 0, eyeW_ = 0, eyeH_ = 0, outW_ = 0;
    std::array<Eye, 2> eye_{};
    GLuint drawFbo_ = 0, readFbo_ = 0, rbo_ = 0;
    std::array<Pbo, 3> pbo_{};
    std::deque<Pbo*> pending_;
    Pbo* mapped_ = nullptr;
};

} // namespace

std::unique_ptr<MirrorBackend> makeOpenVrMirror(const std::filesystem::path& library) {
    return std::make_unique<OpenVrMirror>(library);
}

} // namespace pibridge::record
