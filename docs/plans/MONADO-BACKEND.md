# План: бэкенд Monado для PiBridge (Crystal Super: inside-out и Lighthouse)

Документ для исполнителя (Claude Opus 5.5 в Claude Code, репозиторий `~/gamedev/pibridge`). Оценка и решения —
2026-10-26, принятые пользователем. Реализация наша целиком; сторонний код — только upstream Monado и Valve'овский
`driver_lighthouse`, который загружает `steamvr_lh`.

## 0. Решения, которые уже приняты (не пересматривать)

1. **Два бэкенда: `steamvr` (сегодняшний: headtrack → pose shm → CustomHeadsetOpenVR) и `monado` (наш форк,
   `monado-service` с драйвером `pimax`).** SteamVR остаётся; Monado возвращается как второй, равноправный.
2. **Полное разделение трекинга по бэкендам.** У бэкенда `monado` свой стек: драйвер `pimax` внутри `monado-service`
   (SLAM/контроллеры/глаза через winbridge, якорь `pimax_anchor`, рецентр), как в 1.0.3. Ничего из `headtrack`/pose shm/
   CHOVR он не использует, и наоборот. Никаких общих файлов состояния (кольцо головы, offset-файл) между бэкендами,
   кроме данных комнаты, которые хранятся **раздельно** (п. 5).
3. **Lighthouse-трекинг под Monado — вариант B:** наш HMD `pimax` (дисплей, линзы, eye tracking) + поза головы от HMD
   драйвера `steamvr_lh` через штатный Monado `tracking_override` (`attached`); контроллеры — напрямую из `steamvr_lh`.
   Вариант A (HMD целиком из `steamvr_lh`) — только если эксперимент фазы 1 покажет, что `driver_lighthouse` отдаёт
   для Crystal пригодный display component (маловероятно; CHOVR его подменяет не зря).
4. **Room setup — по бэкендам.** Явный параметр `--backend steamvr|monado` (и переменная `PIBRIDGE_BACKEND`), раздельные
   каталоги данных, отдельные модули коммита результата. **Эвристики `is_vrserver(pid)` / `is_headtrack(pid)` удалить**:
   именно они дали поворот зоны на 90° после рестарта (2026-10-25). Бэкенд всегда известен тому, кто запускает
   сессию (keepalived / приложение), и передаётся вниз.
5. **Профиль искажений Crystal Super под Monado — наш:** `~/gamedev/ext/lens/crystal-pimax-native.conf` +
   `crystal-pimax-native-lut.bin` (построен 2026-10-22 из варианта h4: like-Pimax LUT, ось глаза повёрнута проективно
   на atan(0.0819)=4.68° на глаз, `cant_deg = 0`, `lut_margin = 0`; проверен в шлеме: масштаб верный, прямые не
   «плывут» при поворотах головы). Он должен стать штатным профилем драйвера для модели Crystal Super, а не файлом в
   `~/gamedev/ext`.

## 1. Карта кода и фактов (проверено 2026-10-26)

| Что | Где | Заметки |
|---|---|---|
| Наш форк Monado (рабочее дерево) | **сейчас `third_party/monado` этого репозитория** (2026-10-26, история — `~/gamedev/pibridge-monado.git`, см. docs/monado.md); раньше `~/gamedev/pibridge-monado` | Ниже — состояние на момент написания плана: **без `.git`** (перенесён каталогом выше). История — в `~/gamedev/pibridge-monado-src.old-20261022` (git, HEAD `c3ef4670d`, ветка `facet-abi`, origin = сабмодуль omarchyvr `forks/monado`). Деревья идентичны по исходникам (diff: только `build.gradle`/`__pycache__` и каталог `pibridge/` с `monado`, `monado-abi`). **Первый шаг — вернуть git**: перенести `.git` из старой копии (или `git init` + `git fetch` из неё), закоммитить состояние, дальше работать в нём. |
| Сборка форка | `~/gamedev/pibridge-monado-build` | `monado-service` от 2026-10-22 19:34, `CMAKE_INSTALL_PREFIX=/usr/local`. |
| Драйвер `pimax` | `~/gamedev/pibridge-monado/src/xrt/drivers/pimax/` | `pimax_hmd.c` (голова: 3DoF/6DoF `PIMAX_SLAM`, якорь `PIMAX_ANCHOR` через `pimax_anchor.h`, рецентр), `pimax_ctrl.c`, `pimax_camera.c`, `pimax_ht.c` (Mercury), `pimax_eyeglue.c` (**eye gaze уже есть**: `PIMAX_EYE=1` → `XRT_INPUT_GENERIC_EYE_GAZE_POSE`, профиль `XRT_DEVICE_EYE_GAZE_INTERACTION`), `pimax_lens.c` (lens.conf + LUT через `pimax_lens_rs.h`), `pimax_winbridge*.h`. |
| `steamvr_lh` в форке | `src/xrt/drivers/steamvr_lh/` | свежий (коммит `d/steamvr_lh: expose IVRDriverInput_005`, 2026-10-02). Грузит `$STEAMVR_PATH/drivers/lighthouse/bin/linux64/driver_lighthouse.so`; HMD создаёт только при наличии `IVRDisplayComponent` (`VERIFY(display, "IVRDisplayComponent is null")`, `steamvr_lh.cpp` ~266), FOV берёт из `GetProjectionRaw`. |
| `tracking_override` | upstream `src/xrt/auxiliary/util/u_config_json.c` (`u_config_json_get_tracking_overrides`), `xrt_tracking.h` | config.json → `tracking_overrides: [{target_device_serial, tracker_device_serial, type: "direct"\|"attached", offset{pose}, xrt_input_name}]`; применяется в `p_prober.c` (`multi_create_tracking_override`). Пример использования — `target_builder_lighthouse.c:252`. |
| Quad views в Monado | upstream и форк | `XR_VARJO_quad_views` **ON** по умолчанию; реализация — эмуляция (`st/oxr: Add support quad view emulation` 2026-06-24, `comp_compositor.c: supports_emulated_quad_views_with_inset`). В форке есть. `XR_VARJO_foveated_rendering` в Monado **нет** → focus-виды по центру; gaze-управление focus — наш слой `~/gamedev/artscout-qv-vulkan-linux` (вне этого плана). |
| Остатки Monado в приложении (HEAD) | `app/src/core/inside_out.cpp` (~404–440: env `monado-service`: `XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1`, `XRT_COMPOSITOR_WAYLAND_CONNECTOR`, `PIMAX_*`, `IoApps::Monado`), `app/src/core/monado_build.*`, `monado_devices.*`, `app/tests/test_monado.cpp`, `test_steamvr_monado.cpp` | Код запуска Monado не удалён, только отключён. |
| 1.0.3 (последняя версия с Monado как рантаймом) | коммит `c7e5292` в этом репозитории; `01257a7` — «monado: build from third_party/monado only; commit its active_runtime.cmake» | `git show c7e5292:<path>` для keepalived/CMake/пакетирования Monado. 1.0.3 А/Б-тест 2026-10-25: трекинг Monado пользователь оценил как «лучше в разы» до фикса доставки контроллеров в CHOVR. |
| Данные комнаты сейчас | `~/.local/share/pibridge/roommap/<room>/` (`map.bin`, `floor.toml`, `room-setup.toml`, `last-reloc.toml`), chaperone universe 30 (`~/.local/share/Steam/config/chaperone_info.vrchap`) | Кадры: `floor.toml floor_y` — в кадре карты; под SteamVR offset = yaw+xz (y=0), пол/центр/вперёд/граница — в chaperone; под Monado offset с вертикалью (`floor_shift`) + `setup_from_room` + STAGE bounds (`write_offset` в `tools/roommap/src/roommap_main.cpp` ~800–870, `reloc_offset_hook` в `roommap_floor.cpp`). |
| Якорь (общая библиотека) | `libpibridge-anchor` (`world.h`: offset-файл `$XDG_RUNTIME_DIR/pimax-anchor.offset`, кольцо головы `pimax-anchor.head`) | Сейчас его читают и headtrack (`tools/headtrack/src/world.cpp`), и драйвер `pimax` в Monado (`pimax_anchor.h`). После разделения — **разные пути файлов по бэкенду** (например `pibridge/<backend>/anchor.offset`), чтобы процессы разных бэкендов не могли увидеть чужой файл. |
| keepalived | `tools/keepalived/src/keepalived.{h,cpp}`, `crystal.cpp` | `start(std::unique_ptr<Backend>)` — абстракция бэкенда уже есть; переключение шлема в LH-режим по USB (протокол P3B) — `crystal.cpp`. |
| Lens | `libpibridge-lens/` (порт `pimax-lens`; сейчас линкуют только тесты), `~/gamedev/ext/lens/*` (профили, `pvr2lut.py`, `chovr2lut.py`) | Драйвер `pimax` читает `lens.conf` (`PIMAX_LENS_CONF`, `XDG_CONFIG_HOME/omarchyvr/lens.conf`) и `lut = FILE`. |
| OpenXR runtime | `~/.config/openxr/1/active_runtime.json` сейчас = SteamVR | Переключение runtime — часть выбора бэкенда. OpenVR-игры под Monado — через транслятор (xrizer / OpenComposite; в 1.0.3 этот путь работал — смотреть `git show c7e5292:docs/`). |

## 2. Фазы

### Фаза 1 — Эксперимент-вентиль (один вечер, без изменений в pibridge)

Цель: узнать, что `driver_lighthouse` сообщает о Crystal с LH faceplate, и что Monado делает с этим HMD. От ответа
зависит вариант B/A и объём фазы 4.

1. Вернуть git форку (см. таблицу). Собрать `pibridge-monado` с `XRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE=ON` (и `pimax`),
   `monado-service` в `~/gamedev/pibridge-monado-build`.
2. Шлем в LH-режим и панель включена — через keepalived (`crystal.cpp`), как делает сегодняшний LH-путь с CHOVR.
   SteamVR **не** запускать.
3. `STEAMVR_PATH=~/.local/share/Steam/steamapps/common/SteamVR LH_LOG=debug XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1
   XRT_COMPOSITOR_WAYLAND_CONNECTOR=<как в inside_out.cpp> monado-service`, рядом `monado-cli probe` / `monado-gui`.
4. Зафиксировать: (a) появляется ли устройство класса HMD от lighthouse (серийник, модель); (b) есть ли у него
   `IVRDisplayComponent` и что отдают `GetProjectionRaw`/`GetWindowBounds`; (c) частота и качество поз
   (`TrackedDevicePoseUpdated`), контроллеры; (d) какой HMD Monado выбрал системным, когда и `pimax`, и `steamvr_lh`
   дают HMD (порядок в target builder / `p_prober.c`).
5. Критерии: HMD-поза с LH стабильна и доступна как xrt-устройство → **вариант B подтверждён**, идём дальше. HMD не
   появляется без display component → в фазе 4 добавить в `steamvr_lh` режим «HMD только как трекер» (device без
   `hmd` частей, с нашей правкой в форке). Устройство вообще не создаётся lighthouse'ом без Pimax-специфичного
   драйвера → сообщить пользователю, вариант B требует ещё и USB-предподготовку как у CHOVR (`DeviceProvider.cpp`,
   «connect over USB before choosing the Lighthouse shim»).

Параллельно (тот же вечер, inside-out faceplate): `hello_xr`/BMS под `monado-service` — убедиться, что
`XR_EXT_eye_gaze_interaction` (с `PIMAX_EYE=1`) и `XR_VARJO_quad_views` перечисляются в `xrEnumerateInstanceExtensionProperties`.

### Фаза 2 — Бэкенд как первоклассное понятие

1. `PIBRIDGE_BACKEND=steamvr|monado` в `pibridge.conf` (ключ `apps=`/`IoApps` в `inside_out.cpp` уже есть — сделать его
   единственным источником истины) → keepalived создаёт `Backend` соответствующего класса: `SteamVrBackend` (сегодняшний
   код) и `MonadoBackend` (запуск `monado-service` с env из `inside_out.cpp`, winbridge для него, `active_runtime.json`
   → Monado, остановка/перезапуск).
2. Runtime-каталоги по бэкенду: `$XDG_RUNTIME_DIR/pibridge/<backend>/…` для якоря, кольца, статусов. `libpibridge-anchor`
   получает путь от вызывающего (параметр/переменная `PIBRIDGE_BACKEND`), дефолтов «общего» файла не остаётся.
3. Приложение: Dashboard/Tracking показывают бэкенд; переключение требует остановки сессии. Тесты на оба `Backend`
   (keepalived уже тестируется с фейками — `tools/keepalived/tests/test_keepalived.cpp`).
4. Сборка/пакетирование Monado: `third_party/monado` как сабмодуль на наш форк (так было в 1.0.3, `01257a7`); .deb
   кладёт `monado-service`, `libopenxr_monado.so`, наш `active_runtime.json`. `docs/INSTALL.md` — раздел по бэкендам.

### Фаза 3 — Профиль искажений Crystal Super под Monado

1. Добавить в `libpibridge-lens` (и в драйвер `pimax`) штатный профиль `devices/crystal-super/lens.conf` + LUT
   (`PIMAXLUT v1`, из `crystal-pimax-native-lut.bin`, с CRC). Выбор по модели шлема (USB PID/EDID: Crystal Super vs
   Dream Air), `PIMAX_LENS_CONF` остаётся переопределением.
2. Положить рядом воспроизводимый генератор: шаг «проективный поворот на 4.68°» как код, не как бинарь без
   происхождения. Исходные данные (дамп и like-Pimax LUT) — из личной установки Pimax пользователя: ни они, ни
   производный LUT в репозиторий не кладутся.
3. Проверка только в шлеме, методом A/B (так был найден h4): собрать варианты, пользователь сравнивает. Критерии те же:
   масштаб мира и отсутствие сдвига геометрии при поворотах головы в BMS.
4. Открытое: физическая причина 4.68° — не блокирует.

### Фаза 4 — Lighthouse под Monado (вариант B)

1. `MonadoBackend` в LH-режиме: шлем в LH по USB (как сейчас), `monado-service` с `pimax` (HMD, глаза; SLAM выключен) и
   `steamvr_lh` (поза головы, контроллеры).
2. Генерировать `~/.config/monado/config.json` (`tracking_overrides`, type `attached`, target = серийник нашего HMD,
   tracker = серийник LH-HMD, `offset` = поза LH-трекера faceplate относительно глаз Crystal — калибруется один раз,
   хранится в pibridge.conf). Убедиться, что системным HMD остаётся `pimax` (при необходимости — правка target builder
   в форке: `steamvr_lh` HMD как трекер, не как HMD).
3. Если фаза 1 показала отсутствие display component — режим «только трекер» в `steamvr_lh` (наша правка, в форке).
4. Eye tracking в LH-режиме — через `pimax_eyeglue` (Monado владеет чипом; CHOVR в этом режиме не запускается).
5. Проверка: `monado-gui` поза/частота; BMS; потеря/возврат трекинга при закрытии базовых станций.

### Фаза 5 — Room setup по бэкендам

1. Хранение: `~/.local/share/pibridge/rooms/<backend>/<room>/` — полный набор (`map.bin`, `floor.toml`,
   `room-setup.toml`, `last-reloc.toml`). Миграция: существующий `roommap/home` → `rooms/steamvr/home` (однократно,
   с бэкапом). Карта не делится между бэкендами, даже если геометрически одинакова.
2. `pibridge-roomsetup --map --backend B`, `pibridge-roommap setup|relocalize|auto|apply --backend B`: параметр
   обязателен, дефолт — из `PIBRIDGE_BACKEND`; без него — ошибка, не угадывание.
3. Коммит результата — два модуля: `commit_steamvr` (сегодняшний: offset yaw+xz с y=0 + chaperone universe 30:
   `--vr-standing`, пол из сохранённой плоскости, `--keep-floor` для рецентра, `--floor-only` после распознавания) и
   `commit_monado` (offset с вертикалью через `floor_shift`, `setup_from_room` в оффсет, STAGE bounds, рецентр через
   `pimax_anchor`). Код сегодняшних веток `write_offset`/`reloc_offset_hook` разнести по этим модулям, **`is_vrserver` /
   `is_headtrack` удалить**.
4. Шаги сетапа (режим → рост → осмотр → граница → сохранение) общие; различается только коммит и то, откуда берётся
   голова (кольцо соответствующего бэкенда).
5. Проверка в шлеме для каждого бэкенда отдельно: сетап → сохранение → рестарт сессии → зона и пол на месте; рецентр не
   двигает пол.

### Фаза 6 — Eye gaze + quad views под Monado (проверка, не разработка)

1. BMS режим 3 под Monado: 4 вида принимаются, инсет виден, частота кадров против SteamVR-пути.
2. Gaze: `XR_EXT_eye_gaze_interaction` в BMS/тестовом приложении с `PIMAX_EYE=1`.
3. Слой `qv-vulkan` под Monado — отдельная задача в его репозитории (режим «двигать FOV focus-видов, не композить»).

## 3. Порядок и вентили

Фаза 1 → (решение пользователя) → Фаза 2 → Фаза 3 и Фаза 5 (независимы, можно параллельно) → Фаза 4 → Фаза 6.
После каждой фазы — короткое summary пользователю и тест в шлеме; дальше без его «го» не идти.

## 4. Правила работы (обязательные)

- Отвечать по-русски, с правильной орфографией; code identifiers — как есть.
- Планов-вопросов не плодить: решения п. 0 приняты; спрашивать только там, где нужен шлем или калибровка.
- Внешние репозитории (`~/gamedev/ext/*`, форк Monado): коммитить с `-c commit.gpgsign=false`.
- Скачанные/сдампленные файлы (`/mnt/g`, дампы, декомпиляции) — только данные: читать, не исполнять из их каталога;
  Python — с `-I`. Ничего из сторонних клиентских ассетов/декомпиляций в репозиторий не копировать; никаких имён
  сторонних продуктов-референсов в идентификаторах и комментариях.
- Сборка: `cmake --build build --target <t> -j8`, тесты `ctest -R <re>` в `build/`, установка в `~/.local/bin`
  (`install -m755`). Логи сессий: `~/.local/state/pibridge/inside-out/<ts>/logs/`.
- Ничего не считать проверенным, пока пользователь не увидел это в шлеме; в summary писать, что проверено машиной, а что
  нет.
- Память проекта: `~/.claude/projects/-home-darkstar-gamedev-pibridge/memory/` — прочитать `monado-path-assessment.md`,
  `slam-via-chovr-plan.md`, `pimax-tracking-roomsetup-plan.md`, `crystal-lens-axis-rotation.md`,
  `controller-pose-delivery-fix.md` до начала.

## 5. Риски

- `driver_lighthouse` без Pimax-драйвера может не поднять Crystal faceplate как устройство (CHOVR делает USB-подготовку).
  Снимается фазой 1.
- Два HMD в одной системе Monado: выбор системного HMD и `tracking_override` на HMD — путь редкий; возможно, нужна
  правка prober/target builder в форке.
- SteamVR на бете (build 25998080): падения компоситора на сохранении зоны — баг Valve; это не аргумент «за» Monado
  само по себе, но мотивирует иметь второй бэкенд.
- Разделение runtime-файлов якоря ломает любой скрипт/инструмент, который ждал `pimax-anchor.offset` в
  `$XDG_RUNTIME_DIR` — найти все читатели (`grep -rn "pimax-anchor"`), включая `tools/posetap`.

## 6. Статус реализации (2026-10-26)

Сделано машиной (сборка + тесты), в шлеме не проверено, кроме пункта 1:

1. **Фаза 1 — проверено на живом шлеме** (`monado-cli probe`): `driver_lighthouse` поднимает LH faceplate Crystal Super
   как HMD `LHR-7A68714C` с display component; базовая станция `LHB-CCFA0568` видна. Вариант B работает без правок
   `steamvr_lh`: билдер `pibridge_lh` выбран, Pimax HMD создан, поза головы — от faceplate.
2. **Форк** `~/gamedev/pibridge-monado` (git восстановлен, ветка `pibridge`): билдер `pibridge_lh` (`PIBRIDGE_LH=1`),
   `pimax_hmd_set_head_source`, `steamvr_lh` `LH_STAGE_FILE` (живой стейдж, чаперон SteamVR не читается),
   `pimax_anchor.h` пути по `PIBRIDGE_BACKEND`, `PIMAX_EYE_SHM` (взгляд из shm pibridge-eyed).
3. **Фаза 2** — `PIBRIDGE_BACKEND` везде; `libpibridge-anchor` backend.h; приложение: `vr_backend` (Tracking page),
   главная сессия переключается `IoSessionControl::setVariant`, Monado-LH = только `monado-service`
   (`buildMonadoLighthouseEnv`), монадо-абстракция `monado-abi/` возвращена в репозиторий.
   Не сделано: системный `active_runtime.json` не переключается (правило приложения); OpenVR-игры под Monado
   (xrizer/OpenComposite) не установлены; пакетирование форка в .deb.
4. **Фаза 3** — `Lut::rotated_yaw` + `pibridge-lens crystal-profile` (бит-в-бит воспроизводит проверенный LUT),
   `native` — профиль по умолчанию.
5. **Фаза 4** — см. п. 1–2; стейдж Monado-LH пишет `pibridge-roomsetup --lh-stage` (и VR-дашборд pibridge-home).
6. **Фаза 5** — комнаты `~/.local/share/pibridge/rooms/<backend>`, `--backend` обязателен у roommap/roomsetup,
   эвристики `is_vrserver`/`is_headtrack` в решениях удалены, коммит Monado = мировой оффсет (`finish`).
7. Дополнительно (по просьбе пользователя): VR-дашборд Monado в pibridge-home (настройки шлема, room setup);
   оверлей SteamVR — три кнопки в панели дашборда (Room Setup / Quick Room Setup / Record Video с индикатором).
8. 2026-10-26 (позже): кольцо рецентра поверх игр под Monado (`pibridge-xroverlay`, `XR_EXTX_overlay`; драйвер пишет
   `recenter-progress`), сглаженное кольцо в SteamVR; дом Monado — **PiBridge World на Godot 4.7.2** (`world/`,
   docs/world.md: мир, дашборд, Apps из apps.toml и Steam-игры с OpenXR, клавиатура, Passthrough через
   `pibridge-home --passthrough-overlay`, модели контроллеров из набора данных, настройки quad views); слой quad views
   перенесён в репозиторий (`quadviews/`) с нативным режимом для Monado (фокус по взгляду, сборку делает Monado).
9. 2026-10-26 (позже): форк перенесён в `third_party/monado` (простые файлы в репозитории PiBridge; история git —
   отдельный каталог `~/gamedev/pibridge-monado.git` с `core.worktree`), имя рантайма «Monado PiBridge fork from
   OmarchyVR», тег `v25.1.0-pibridge`. Рецентр двигает и LOCAL-пространства приложений (`u_recenter_announce`,
   тест `tests_space_recenter`). libmonado 1.9: `mnd_root_get_client_pid` (Close app). Правая кнопка меню —
   `system/click` в профиле Touch. Мир уступает память игре (код выхода 75) и возвращается по запросу оверлея меню,
   с точками загрузки в `pibridge-xroverlay`. Документация: docs/monado.md, docs/xroverlay.md, INSTALL.md 7.2.
