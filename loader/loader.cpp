// pibridge loader-оракул: грузит настоящие трекинговые DLL Pimax под Wine на Linux
// и даёт дёргать их C-ABI, чтобы (а) сверять с нашим C++-портом и (б) работать
// helper-сервисом, пока порт зреет (hybrid-стратегия).
//
// Режимы (argv[1]):
//   smoke  — загрузить SLAM + контроллеры, разрезолвить экспорты, *GetVersion.
//   init   — то же + реальная инициализация движка (*InitialImpl) с живыми
//            путями из /mnt/c (через Wine-диск Z:). Проверяет, что SLAM
//            поднимается под Wine (внутри OpenGL/GLEW, загрузка 111МБ platMap).
// Дальше: подача записанного take (Put*CameraImage / Put*ImuData) -> позы в CSV,
// потом IPC-обёртка как постоянный сервис для pibridge/Monado.
//
// ABI из re-dll/README.md + frida-pass2.json. На x64 Windows одно соглашение
// вызова, MinGW совпадает. Хвостовые "аргументы" из Frida (0x9, install-path)
// повторяются у всех функций -> это мусор со стека, в сигнатуры не входят.

#include <windows.h>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>

// Локальный kit в ~/.local/share/pibridge/windata (dlls/ + slam/), адресуемый через
// Wine-диск Z: (Z: -> /). База вычисляется из $HOME, переопределяется argv[2].
static char g_base[MAX_PATH] = "";  // $PIBRIDGE_WINDATA, домашняя папка Wine (build_default_base) или argv[2]

// $PIBRIDGE_WINDATA, иначе <домашняя папка>\.local\share\pibridge\windata; домашнюю папку Wine даёт в
// $WINEHOMEDIR ("\??\Z:\home\<user>"), HOME в Windows-процесс не передаётся
static void build_default_base(void) {
    if (const char* kit = getenv("PIBRIDGE_WINDATA"); kit && *kit) {
        snprintf(g_base, sizeof(g_base), "%s", kit);
        return;
    }
    const char* wh = getenv("WINEHOMEDIR");
    if (!wh || !*wh) return;
    if (strncmp(wh, "\\??\\", 4) == 0) wh += 4;
    snprintf(g_base, sizeof(g_base), "%s\\.local\\share\\pibridge\\windata", wh);
}

static const char* kSlamDll = "pimax_slam.pi.dll";
static const char* kCtrlDll = "LedObjectPoseEstimator.dll";

static const char* kSlamExports[] = {
    "HeadsetInitialImpl", "HeadsetReleaseImpl", "HeadsetGetVersion",
    "HeadsetSetTrackingMode", "HeadsetSetDeviceEnable", "HeadsetPutCameraImage",
    "HeadsetPutHmdImuData", "HeadsetLeftImageNumber", "HeadsetGetGroundState",
    "HeadsetGetLocModeState",
};
static const char* kCtrlExports[] = {
    "ControllerInitialImpl", "ControllerReleaseImpl", "ControllerGetVersion",
    "ControllerSetDeviceEnable", "ControllerAddObject", "ControllerRemoveObject",
    "ControllerPutCameraImage", "ControllerPutControllerImuData",
};

typedef const char* (*GetVersionFn)(void);
typedef int (*HeadsetInitialFn)(const char* calib, const char* dataDir,
                                const char* vocab, void* p4, void* p5);
typedef int (*ControllerInitialFn)(const char* calib, const char* dataDir,
                                   const char* db, void* p4, void* p5);
typedef int (*ReleaseFn)(void);
// HeadsetSetTrackingMode(byte) / HeadsetSetDeviceEnable(byte) — один байтовый аргумент
// (дизассемблирование: mov [rsp+8],cl; -1 если движок не инициализирован).
typedef int (*SetByteFn)(int);
// HeadsetPutCameraImage(cam0..3, dt) — 4 структуры камер + float; 25 Гц.
typedef int (*PutCamFn)(void*, void*, void*, void*, float);

// ---- контроллеры (LedObjectPoseEstimator.dll), по данным frida pass1-3 ----
// LED-кадр (короткая выдержка, чередуется со SLAM-кадрами): {u64 ts; u32 shutter_ns; u32 idx; w; h; stride; pad; px}.
struct LedFrame {
    uint64_t ts;
    uint32_t shutter_ns;  // 40000 на Windows
    uint32_t idx;
    uint32_t width, height, stride, pad;
    uint8_t* pixels;
};
static_assert(sizeof(LedFrame) == 40, "LedFrame должен быть 40 байт");
// Поза головы на момент LED-кадра в осях LED-движка (x, z, -y относительно SLAM): quat x,y,z,w + pos.
struct HeadPoseForLed { float q[4]; float p[3]; float pad[9]; };
// Светодиод модели: {int id; pos[3]; normal[3]} (controller_params.json ModelPoints).
struct LedRec { int32_t id; float pos[3]; float normal[3]; };
// std::string MSVC x64 с короткой строкой (SSO): буфер 16 байт, размер, ёмкость 15.
struct MsvcString { char buf[16]; uint64_t size; uint64_t cap; };
typedef int (*CtrlAddFn)(int idx, int n, const LedRec* leds, const float* imu_to_dev, int a4, int a5, int a6,
                         const MsvcString* sub);
typedef int (*CtrlPutCamFn)(void*, void*, void*, void*, const HeadPoseForLed*, void*);
typedef int (*CtrlPutImuFn)(int idx, void* imu, float* out, float* scratch);

// controller_params.json -> рука (0 левая, 1 правая), 15 светодиодов, ImuToDevice как quat(x,y,z,w)+сдвиг.
struct CtrlModel { int hand = -1; int n = 0; LedRec leds[32]; float imu7[8] = {0}; };
static bool numbers_after(const char* s, const char* key, double* out, int count) {
    const char* p = strstr(s, key);
    if (!p) return false;
    p = strchr(p, '[');
    if (!p) return false;
    ++p;
    for (int i = 0; i < count; ++i) {
        char* e = nullptr;
        out[i] = strtod(p, &e);
        if (e == p) return false;
        p = e;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p;
    }
    return true;
}
static bool load_ctrl_model(const char* path, CtrlModel& m) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    static char buf[16384];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = 0;
    m.hand = strstr(buf, "\"Right\"") ? 1 : (strstr(buf, "\"Left\"") ? 0 : -1);
    m.n = 0;
    for (int i = 0; i < 32; ++i) {
        char key[32];
        snprintf(key, sizeof(key), "\"Point%d\"", i);
        double v[6];
        if (!numbers_after(buf, key, v, 6)) break;
        m.leds[m.n] = LedRec{i, {(float)v[0], (float)v[1], (float)v[2]}, {(float)v[3], (float)v[4], (float)v[5]}};
        ++m.n;
    }
    double t[16];
    if (!numbers_after(buf, "\"ImuToDevice\"", t, 16)) return false;
    // матрица (row-major) -> кватернион (x,y,z,w), сдвиг из 4-го столбца
    double tr = t[0] + t[5] + t[10], qw, qx, qy, qz;
    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2;
        qw = 0.25 * s; qx = (t[9] - t[6]) / s; qy = (t[2] - t[8]) / s; qz = (t[4] - t[1]) / s;
    } else if (t[0] > t[5] && t[0] > t[10]) {
        double s = sqrt(1.0 + t[0] - t[5] - t[10]) * 2;
        qw = (t[9] - t[6]) / s; qx = 0.25 * s; qy = (t[1] + t[4]) / s; qz = (t[2] + t[8]) / s;
    } else if (t[5] > t[10]) {
        double s = sqrt(1.0 + t[5] - t[0] - t[10]) * 2;
        qw = (t[2] - t[8]) / s; qx = (t[1] + t[4]) / s; qy = 0.25 * s; qz = (t[6] + t[9]) / s;
    } else {
        double s = sqrt(1.0 + t[10] - t[0] - t[5]) * 2;
        qw = (t[4] - t[1]) / s; qx = (t[2] + t[8]) / s; qy = (t[6] + t[9]) / s; qz = 0.25 * s;
    }
    float imu7[8] = {(float)qx, (float)qy, (float)qz, (float)qw, (float)t[3], (float)t[7], (float)t[11], 0.0f};
    memcpy(m.imu7, imu7, sizeof(imu7));
    return m.hand >= 0 && m.n > 0;
}
// HeadsetPutHmdImuData(imuIn, outPose) — пишет позу в outPose; ~1 кГц.
typedef int (*PutImuFn)(void*, void*);

// Камера-структура 40 байт (re-dll/README.md, Frida pass1).
struct CamFrame {
    uint64_t ts;         // @0  временная метка
    uint64_t shutter_ns; // @8  выдержка в нс (0 -> SLAM отбрасывает кадр: "too short")
    uint32_t idx;      // @16 индекс камеры 0..3
    uint32_t width;    // @20 640
    uint32_t height;   // @24 480
    uint32_t stride;   // @28 2560 (комбинированный кадр 4x640)
    uint8_t* pixels;   // @32 указатель на пиксели
};
static_assert(sizeof(CamFrame) == 40, "CamFrame должен быть 40 байт");

// IMU-сэмпл головы: u64 ts, затем float accel[3], gyro[3] (re-dll/README pass1).
struct ImuSample {
    uint64_t ts;
    float accel[3];
    float gyro[3];
    float pad[8];
};

static HMODULE load_dll(const char* dir, const char* name) {
    char path[MAX_PATH];
    snprintf(path, sizeof(path), "%s\\%s", dir, name);
    HMODULE h = LoadLibraryA(path);
    if (!h) fprintf(stderr, "  [FAIL] LoadLibrary %s -> код %lu\n", path, GetLastError());
    return h;
}

static int resolve_all(HMODULE h, const char* const* names, int n) {
    int ok = 0;
    for (int i = 0; i < n; ++i) {
        FARPROC p = GetProcAddress(h, names[i]);
        printf("    %-32s %s\n", names[i], p ? "ok" : "НЕ НАЙДЕН");
        if (p) ++ok;
    }
    return ok;
}

static void call_version(HMODULE h, const char* sym) {
    GetVersionFn fn = (GetVersionFn)GetProcAddress(h, sym);
    if (!fn) { printf("  %s: нет символа\n", sym); return; }
    const char* v = fn();
    if (v && !IsBadStringPtrA(v, 64)) printf("  %s -> \"%s\"\n", sym, v);
    else printf("  %s -> raw 0x%016llx (не строка)\n", sym, (unsigned long long)(uintptr_t)v);
}

int main(int argc, char** argv) {
    const char* mode = (argc > 1) ? argv[1] : "smoke";
    build_default_base();
    bool isPlay = (strcmp(mode, "play") == 0);
    const bool isCtrlPlay = (strcmp(mode, "ctrlplay") == 0);
    const bool takesFiles = isPlay || isCtrlPlay;
    const char* replayPath = (takesFiles && argc > 2) ? argv[2] : nullptr;
    const char* outCsv     = (takesFiles && argc > 3) ? argv[3] : nullptr;  // ctrlplay: префикс, + "-left.csv"
    if (!takesFiles && argc > 2) snprintf(g_base, sizeof(g_base), "%s", argv[2]);

    // Layout kit: base\dlls (DLL), base\slam (словарь/база/калибровка/карта).
    char runtime[MAX_PATH], slamData[MAX_PATH], calib[MAX_PATH], vocab[MAX_PATH], db[MAX_PATH];
    snprintf(runtime,  sizeof(runtime),  "%s\\dlls", g_base);
    snprintf(slamData, sizeof(slamData), "%s\\slam", g_base);
    {   // калибровка шлема: первый device_calibration_<серийник>.xml в slam/
        char pat[MAX_PATH];
        snprintf(pat, sizeof(pat), "%s\\device_calibration_*.xml", slamData);
        WIN32_FIND_DATAA fd;
        HANDLE fh = FindFirstFileA(pat, &fd);
        if (fh != INVALID_HANDLE_VALUE) { FindClose(fh); snprintf(calib, sizeof(calib), "%s\\%s", slamData, fd.cFileName); }
        else snprintf(calib, sizeof(calib), "%s\\device_calibration.xml", slamData);
    }
    snprintf(vocab,    sizeof(vocab),    "%s\\voc_GEN_8X4.dbow", slamData);
    snprintf(db,       sizeof(db),       "%s\\pimax_database.bin", slamData);

    printf("pibridge loader-оракул | режим: %s | база: %s\n\n", mode, g_base);
    SetDllDirectoryA(runtime);  // чтобы opencv_*453/ceres/msvcp140 находились рядом

    printf("[SLAM] %s\n", kSlamDll);
    HMODULE slam = load_dll(runtime, kSlamDll);
    if (!slam) return 1;
    resolve_all(slam, kSlamExports, (int)(sizeof(kSlamExports)/sizeof(*kSlamExports)));
    call_version(slam, "HeadsetGetVersion");

    printf("\n[CTRL] %s\n", kCtrlDll);
    HMODULE ctrl = load_dll(runtime, kCtrlDll);
    if (!ctrl) return 1;
    resolve_all(ctrl, kCtrlExports, (int)(sizeof(kCtrlExports)/sizeof(*kCtrlExports)));
    call_version(ctrl, "ControllerGetVersion");

    const bool isCtrl = (strcmp(mode, "ctrl") == 0) || isCtrlPlay;
    bool doInit = (strcmp(mode, "init") == 0) || (strcmp(mode, "feed") == 0) || isPlay || isCtrl;
    bool doFeed = (strcmp(mode, "feed") == 0);
    if (!doInit) {
        printf("\nИТОГ (smoke): обе DLL загружены, экспорты на месте\n");
        return 0;
    }

    // --- режим init: реальная инициализация движка ---
    printf("\n=== init ===\n");
    printf("calib : %s\n", calib);
    printf("data  : %s\n", slamData);
    printf("vocab : %s\n", vocab);
    printf("db    : %s\n\n", db);

    HeadsetInitialFn HeadsetInitial = (HeadsetInitialFn)GetProcAddress(slam, "HeadsetInitialImpl");
    printf("HeadsetInitialImpl ... "); fflush(stdout);
    int rh = HeadsetInitial(calib, slamData, vocab, nullptr, nullptr);
    printf("ret=%d\n", rh);

    ControllerInitialFn ControllerInitial = (ControllerInitialFn)GetProcAddress(ctrl, "ControllerInitialImpl");
    printf("ControllerInitialImpl ... "); fflush(stdout);
    int rc = ControllerInitial(calib, slamData, db, nullptr, nullptr);
    printf("ret=%d\n", rc);

    // Включить трекинг (как pi_server: SetTrackingMode -> SetDeviceEnable).
    SetByteFn SetMode   = (SetByteFn)GetProcAddress(slam, "HeadsetSetTrackingMode");
    SetByteFn SetEnable = (SetByteFn)GetProcAddress(slam, "HeadsetSetDeviceEnable");
    int tmode = 0; const char* tm = getenv("PIBRIDGE_TMODE"); if (tm) tmode = atoi(tm);
    if (SetMode)   printf("HeadsetSetTrackingMode(%d) ret=%d\n", tmode, SetMode(tmode));
    if (SetEnable) printf("HeadsetSetDeviceEnable(1) ret=%d\n", SetEnable(1));

    printf("\nдвижок инициализирован, 1 с на фоновые потоки...\n");
    fflush(stdout);
    Sleep(1000);

    if (doFeed) {
        printf("\n=== feed (синтетика) ===\n"); fflush(stdout);
        PutCamFn PutCam = (PutCamFn)GetProcAddress(slam, "HeadsetPutCameraImage");
        PutImuFn PutImu = (PutImuFn)GetProcAddress(slam, "HeadsetPutHmdImuData");
        printf("PutCameraImage=%p PutHmdImuData=%p\n", (void*)PutCam, (void*)PutImu);

        // Один комбинированный кадр 2560x480, 4 камеры = окна 640 со сдвигом.
        const uint32_t W = 640, H = 480, STRIDE = 2560;
        static uint8_t buf[STRIDE * H];
        for (uint32_t y = 0; y < H; ++y)
            for (uint32_t x = 0; x < STRIDE; ++x)
                buf[y * STRIDE + x] = (uint8_t)((x ^ y) & 0xff);  // неоднородный паттерн
        CamFrame cam[4];
        for (int i = 0; i < 4; ++i) {
            cam[i] = CamFrame{};
            cam[i].idx = (uint32_t)i;
            cam[i].width = W; cam[i].height = H; cam[i].stride = STRIDE;
            cam[i].pixels = buf + (uint32_t)i * W;  // сдвиг окна камеры
        }

        // outPose: с запасом, обнулить, искать запись.
        float outPose[32];
        memset(outPose, 0, sizeof(outPose));

        const uint64_t NS = 1000000;          // шаг IMU = 1 мс (~1 кГц)
        uint64_t t = 1000000000ULL;
        int wrote = 0, camCalls = 0, imuCalls = 0;
        for (int step = 0; step < 400; ++step) {
            if (PutCam && (step % 40 == 0)) {  // ~25 Гц относительно 1 кГц IMU
                for (int i = 0; i < 4; ++i) cam[i].ts = t;
                PutCam(&cam[0], &cam[1], &cam[2], &cam[3], 0.019f);
                ++camCalls;
            }
            if (PutImu) {
                ImuSample s{}; s.ts = t;
                s.accel[0] = 0.0f; s.accel[1] = 0.0f; s.accel[2] = 9.81f;  // покой
                s.gyro[0] = s.gyro[1] = s.gyro[2] = 0.0f;
                memset(outPose, 0, sizeof(outPose));
                int r = PutImu(&s, outPose);
                ++imuCalls;
                if (step == 399) {  // последняя поза
                    printf("PutHmdImuData ret=%d, outPose[0..12]:\n  ", r);
                    for (int k = 0; k < 13; ++k) printf("%.5f ", outPose[k]);
                    printf("\n");
                }
                for (int k = 0; k < 16; ++k) if (outPose[k] != 0.0f) { wrote = 1; break; }
            }
            t += NS;
        }
        printf("вызовов: cam=%d imu=%d; outPose %s\n",
               camCalls, imuCalls, wrote ? "ЗАПИСАН (ненулевой)" : "остался нулевым");
        float q0 = outPose[0], q1 = outPose[1], q2 = outPose[2], q3 = outPose[3];
        printf("|quat| = %.4f\n", (double)(q0*q0 + q1*q1 + q2*q2 + q3*q3));
        fflush(stdout);
    }

    if (isCtrl) {
        // Смоук контроллеров: SetDeviceEnable(1), AddObject x2 из H1\*\controller_params.json (как pi_server),
        // синтетические LED-кадры с позой головы, IMU с гравитацией; пишется ли поза и не падает ли.
        printf("\n=== ctrl (синтетика) ===\n"); fflush(stdout);
        SetByteFn cEnable = (SetByteFn)GetProcAddress(ctrl, "ControllerSetDeviceEnable");
        CtrlAddFn cAdd = (CtrlAddFn)GetProcAddress(ctrl, "ControllerAddObject");
        CtrlPutCamFn cCam = (CtrlPutCamFn)GetProcAddress(ctrl, "ControllerPutCameraImage");
        CtrlPutImuFn cImu = (CtrlPutImuFn)GetProcAddress(ctrl, "ControllerPutControllerImuData");
        printf("ControllerSetDeviceEnable(1) ret=%d\n", cEnable(1)); fflush(stdout);
        char pat[MAX_PATH];
        snprintf(pat, sizeof(pat), "%s\\H1\\*", slamData);
        WIN32_FIND_DATAA fd;
        HANDLE fh = FindFirstFileA(pat, &fd);
        int added = 0;
        const MsvcString sub = {"\\H1\\", 4, 15};
        while (fh != INVALID_HANDLE_VALUE) {
            if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && fd.cFileName[0] != '.') {
                char path[MAX_PATH];
                snprintf(path, sizeof(path), "%s\\H1\\%s\\controller_params.json", slamData, fd.cFileName);
                CtrlModel m;
                if (load_ctrl_model(path, m)) {
                    printf("ControllerAddObject(hand=%d, %d LEDs, imu q=(%.4f %.4f %.4f %.4f) t=(%.5f %.5f %.5f)) ... ",
                           m.hand, m.n, m.imu7[0], m.imu7[1], m.imu7[2], m.imu7[3], m.imu7[4], m.imu7[5], m.imu7[6]);
                    fflush(stdout);
                    printf("ret=%d\n", cAdd(m.hand, m.n, m.leds, m.imu7, 2, 0, 0x34, &sub));
                    ++added;
                }
            }
            if (!FindNextFileA(fh, &fd)) { FindClose(fh); break; }
        }
        printf("моделей: %d\n", added); fflush(stdout);

        if (isCtrlPlay) {
            // Запись (winbridge/ctrl_prep.cpp, "CREP"): LED-кадры + IMU обеих рук, в темпе записи (движок
            // работает на своём потоке). Поза головы единичная: поза контроллера выходит относительно шлема.
            FILE* f = replayPath ? fopen(replayPath, "rb") : nullptr;
            if (!f) { fprintf(stderr, "не открыть %s\n", replayPath ? replayPath : "(нет)"); return 2; }
            char magic[4]; uint32_t ver = 0;
            if (fread(magic, 1, 4, f) != 4 || fread(&ver, 4, 1, f) != 1 || memcmp(magic, "CREP", 4) != 0) {
                fprintf(stderr, "не CREP\n"); return 2;
            }
            FILE* csv[2] = {nullptr, nullptr};
            for (int h = 0; h < 2; ++h) {
                char p[MAX_PATH];
                snprintf(p, sizeof(p), "%s-%s.csv", outCsv ? outCsv : "ctrlpose", h ? "right" : "left");
                csv[h] = fopen(p, "w");
                if (csv[h]) fprintf(csv[h], "ts,q0,q1,q2,q3,px,py,pz,w0,w1,w2,v0,v1,v2\n");
            }
            static uint8_t tiles[4][640 * 480];
            LedFrame lf[4];
            HeadPoseForLed hp{};
            hp.q[3] = 1.0f;
            float out[2][32] = {{0}}, scratch[2][32] = {{0}};
            long nLed = 0, nImu[2] = {0, 0}, nPos[2] = {0, 0};
            ULONGLONG wall0 = GetTickCount64();
            uint64_t t0 = 0;
            for (;;) {
                uint8_t typ;
                if (fread(&typ, 1, 1, f) != 1) break;
                if (typ == 1) {
                    uint64_t ts; uint32_t sh;
                    if (fread(&ts, 8, 1, f) != 1 || fread(&sh, 4, 1, f) != 1 || fread(tiles, 1, sizeof(tiles), f) != sizeof(tiles)) break;
                    if (t0 == 0) t0 = ts;
                    const double simMs = (double)(ts - t0) / 1e6;
                    const ULONGLONG now = GetTickCount64() - wall0;
                    if (simMs > (double)now) Sleep((DWORD)(simMs - (double)now));
                    for (int i = 0; i < 4; ++i) lf[i] = LedFrame{ts, sh, (uint32_t)i, 640, 480, 640, 0, tiles[i]};
                    cCam(&lf[0], &lf[1], &lf[2], &lf[3], &hp, nullptr);
                    ++nLed;
                } else if (typ == 2) {
                    uint32_t hand; uint64_t ts; float a[3], g[3];
                    if (fread(&hand, 4, 1, f) != 1 || fread(&ts, 8, 1, f) != 1 || fread(a, 4, 3, f) != 3 || fread(g, 4, 3, f) != 3) break;
                    if (hand > 1) continue;
                    if (t0 == 0) t0 = ts;
                    ImuSample s{}; s.ts = ts;
                    memcpy(s.accel, a, sizeof(a)); memcpy(s.gyro, g, sizeof(g));
                    memset(out[hand], 0, sizeof(out[hand]));
                    cImu((int)hand, &s, out[hand], scratch[hand]);
                    ++nImu[hand];
                    const float* o = out[hand];
                    if (o[4] != 0.0f || o[5] != 0.0f || o[6] != 0.0f) ++nPos[hand];
                    if (csv[hand]) {
                        fprintf(csv[hand], "%llu", (unsigned long long)ts);
                        for (int k = 0; k < 13; ++k) fprintf(csv[hand], ",%.6f", o[k]);
                        fprintf(csv[hand], "\n");
                    }
                }
            }
            fclose(f);
            for (int h = 0; h < 2; ++h) if (csv[h]) fclose(csv[h]);
            printf("ctrlplay: LED-кадров %ld; IMU левый %ld (с позицией %ld), правый %ld (с позицией %ld)\n", nLed,
                   nImu[0], nPos[0], nImu[1], nPos[1]);
            fflush(stdout);
        } else {
        static uint8_t led[4][640 * 480];  // тёмные кадры: светодиодов нет
        LedFrame lf[4];
        HeadPoseForLed hp{};
        hp.q[3] = 1.0f;
        float out[2][32] = {{0}}, scratch[2][32] = {{0}};
        uint64_t t = 1000000000ULL;
        int camCalls = 0;
        for (int step = 0; step < 400; ++step) {          // 1 кГц шаг, IMU каждой руки 500 Гц
            if (step % 40 == 20) {                        // LED-кадр между SLAM-кадрами, 25 Гц
                for (int i = 0; i < 4; ++i)
                    lf[i] = LedFrame{t, 40000, (uint32_t)i, 640, 480, 640, 0, led[i]};
                cCam(&lf[0], &lf[1], &lf[2], &lf[3], &hp, nullptr);
                ++camCalls;
            }
            if (step % 2 == 0) {
                for (int hand = 0; hand < 2; ++hand) {
                    ImuSample s{}; s.ts = t;
                    s.accel[2] = 9.81f;                  // покой
                    cImu(hand, &s, out[hand], scratch[hand]);
                }
            }
            t += 1000000;
        }
        for (int hand = 0; hand < 2; ++hand) {
            const float* o = out[hand];
            printf("рука %d: out[0..12] =", hand);
            for (int k = 0; k < 13; ++k) printf(" %.4f", o[k]);
            printf("   |q|=%.4f\n", (double)(o[0]*o[0] + o[1]*o[1] + o[2]*o[2] + o[3]*o[3]));
        }
        printf("LED-кадров: %d (6-й аргумент NULL)\n", camCalls);
        fflush(stdout);
        }
    }

    if (isPlay) {
        printf("\n=== play: %s ===\n", replayPath ? replayPath : "(нет файла)");
        PutCamFn PutCam = (PutCamFn)GetProcAddress(slam, "HeadsetPutCameraImage");
        PutImuFn PutImu = (PutImuFn)GetProcAddress(slam, "HeadsetPutHmdImuData");
        FILE* f = replayPath ? fopen(replayPath, "rb") : nullptr;
        FILE* csv = outCsv ? fopen(outCsv, "w") : nullptr;
        if (!f) { fprintf(stderr, "не открыть replay\n"); return 2; }
        char magic[4]; uint32_t ver, W, H, ncam;
        fread(magic, 1, 4, f); fread(&ver, 4, 1, f); fread(&W, 4, 1, f);
        fread(&H, 4, 1, f); fread(&ncam, 4, 1, f);
        printf("replay: magic=%.4s ver=%u %ux%u ncam=%u\n", magic, ver, W, H, ncam);
        if (csv) fprintf(csv, "ts,q0,q1,q2,q3,px,py,pz\n");

        static uint8_t c0[640*480], c1[640*480];
        float outPose[32];
        CamFrame cam[4];
        int imuN = 0, camN = 0, nz = 0;
        float lastPose[8] = {0};
        // Темп реального времени: SLAM обрабатывает кадры на фоновом потоке,
        // поэтому подаём их не быстрее записи (иначе поток не локализуется).
        double speed = 1.0; const char* sp = getenv("PIBRIDGE_SPEED"); if (sp) speed = atof(sp);
        uint64_t shutterNs = 10000000; const char* sh = getenv("PIBRIDGE_SHUTTER");
        if (sh) shutterNs = strtoull(sh, nullptr, 10);
        ULONGLONG wall0 = GetTickCount64(); uint64_t t0 = 0; bool haveT0 = false;
        while (true) {
            uint8_t typ;
            if (fread(&typ, 1, 1, f) != 1) break;
            if (typ == 0) {
                uint64_t ts; float v[6];
                fread(&ts, 8, 1, f); fread(v, 4, 6, f);
                if (!haveT0) { t0 = ts; haveT0 = true; }
                ImuSample s{}; s.ts = ts;
                s.accel[0]=v[0]; s.accel[1]=v[1]; s.accel[2]=v[2];
                s.gyro[0]=v[3];  s.gyro[1]=v[4];  s.gyro[2]=v[5];
                memset(outPose, 0, sizeof(outPose));
                if (PutImu) PutImu(&s, outPose);
                ++imuN;
                if (csv) fprintf(csv, "%llu,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n",
                    (unsigned long long)ts, outPose[0],outPose[1],outPose[2],outPose[3],
                    outPose[4],outPose[5],outPose[6]);
                for (int k=4;k<7;++k) if (outPose[k]!=0.0f) { nz=1; break; }
                for (int k=0;k<7;++k) lastPose[k]=outPose[k];
            } else {
                uint64_t ts;
                fread(&ts, 8, 1, f);
                fread(c0, 1, W*H, f); fread(c1, 1, W*H, f);
                if (!haveT0) { t0 = ts; haveT0 = true; }
                // держать темп реального времени к моменту кадра
                double simMs = (double)(ts - t0) / 1e6 / speed;
                ULONGLONG now = GetTickCount64() - wall0;
                if (simMs > (double)now) Sleep((DWORD)(simMs - (double)now));
                for (int i=0;i<4;++i){ cam[i]=CamFrame{}; cam[i].idx=(uint32_t)i;
                    cam[i].width=W; cam[i].height=H; cam[i].stride=W; cam[i].ts=ts;
                    cam[i].shutter_ns=shutterNs; }
                cam[0].pixels=c0; cam[1].pixels=c1; cam[2].pixels=c0; cam[3].pixels=c1;
                if (PutCam) PutCam(&cam[0],&cam[1],&cam[2],&cam[3], 0.019f);
                ++camN;
                if (camN % 50 == 0) { printf("  кадр %d: pos=(%.3f,%.3f,%.3f) |q|=%.3f\n",
                    camN, lastPose[4],lastPose[5],lastPose[6],
                    (double)(lastPose[0]*lastPose[0]+lastPose[1]*lastPose[1]+lastPose[2]*lastPose[2]+lastPose[3]*lastPose[3]));
                    fflush(stdout); }
            }
        }
        fclose(f); if (csv) fclose(csv);
        printf("play: imu=%d cam=%d; позиция %s; CSV=%s\n",
               imuN, camN, nz ? "НЕНУЛЕВАЯ (движок локализуется)" : "осталась нулевой",
               outCsv ? outCsv : "(нет)");
        fflush(stdout);
    }

    ReleaseFn HeadsetRelease = (ReleaseFn)GetProcAddress(slam, "HeadsetReleaseImpl");
    ReleaseFn ControllerRelease = (ReleaseFn)GetProcAddress(ctrl, "ControllerReleaseImpl");
    if (ControllerRelease) ControllerRelease();
    if (HeadsetRelease) HeadsetRelease();
    printf("ИТОГ (init): движок поднят и остановлен\n");
    return 0;
}
