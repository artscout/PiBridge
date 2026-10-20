// SPDX-License-Identifier: MIT
// Tests of the Apps page's logic: the KeyValues (VDF/ACF) reader, the Steam library scan (a synthetic Steam root in a
// temporary directory; never the user's), apps.toml (keeps what it does not know, atomic writes), the launcher
// (FakeRunner + FakeAppProcesses: nothing is ever started) and the "SteamVR fully ready" state machine (injected clock).
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "core/apps_launch.h"
#include "core/apps_own.h"
#include "core/apps_runtime.h"
#include "core/apps_steam.h"
#include "core/apps_vdf.h"
#include "core/runner.h"
#include "core/steamvr.h"
#include "core/steamvr_ready.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

void writeAll(const QString& p, const QByteArray& b) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
}

QByteArray readAll(const QString& p) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
}

// Synthetic files in Steam's layout (made-up ids, names and paths).
const char* kLibraryFolders = R"("libraryfolders"
{
	"0"
	{
		"path"		"@ROOT@"
		"label"		""
		"apps"
		{
			"900001"		"123"
			"250820"		"456"
		}
	}
	"1"
	{
		"path"		"@LIB2@"
		"label"		"Games"
	}
	"2"
	{
		"path"		"/nonexistent/library"
	}
}
)";

QByteArray acf(const QString& id, const QString& name, int flags, qint64 played) {
    return QStringLiteral("\"AppState\"\n{\n\t\"appid\"\t\t\"%1\"\n\t\"Universe\"\t\t\"1\"\n\t\"name\"\t\t\"%2\"\n"
                          "\t\"StateFlags\"\t\t\"%3\"\n\t\"installdir\"\t\t\"%2\"\n\t\"LastPlayed\"\t\t\"%4\"\n"
                          "\t\"InstalledDepots\"\n\t{\n\t\t\"%1\"\n\t\t{\n\t\t\t\"manifest\"\t\t\"1\"\n\t\t}\n\t}\n}\n")
        .arg(id, name)
        .arg(flags)
        .arg(played)
        .toUtf8();
}

const char* kVrManifest = R"({
	"source": "steam",
	"applications": [
		{ "app_key": "steam.app.900001", "launch_type": "url", "last_played_time": "1800000500",
		  "url": "steam://launch/900001/VR" },
		{ "app_key": "steam.overlay.900004", "last_played_time": 1700000000, "url": "steam://launch/900004/openvroverlay" },
		{ "app_key": "openvr.tool.steamvr_room_setup" }
	]
})";

struct FakeSteam {
    QTemporaryDir tmp;
    QString root, lib2;
    FakeSteam() {
        root = tmp.path() + QStringLiteral("/Steam");
        lib2 = tmp.path() + QStringLiteral("/lib2");
        QByteArray lf = kLibraryFolders;
        lf.replace("@ROOT@", root.toUtf8()).replace("@LIB2@", lib2.toUtf8());
        writeAll(root + QStringLiteral("/steamapps/libraryfolders.vdf"), lf);
        writeAll(root + QStringLiteral("/steamapps/appmanifest_900001.acf"), acf(QStringLiteral("900001"), QStringLiteral("Sky Racer"), 4, 1800000000));
        writeAll(root + QStringLiteral("/steamapps/appmanifest_250820.acf"), acf(QStringLiteral("250820"), QStringLiteral("SteamVR"), 4, 1800000900));
        writeAll(root + QStringLiteral("/steamapps/appmanifest_1493710.acf"), acf(QStringLiteral("1493710"), QStringLiteral("Proton Experimental"), 4, 0));
        writeAll(root + QStringLiteral("/steamapps/appmanifest_999999.acf"), acf(QStringLiteral("999999"), QStringLiteral("Proton 99.0"), 4, 0));
        writeAll(root + QStringLiteral("/steamapps/appmanifest_900002.acf"), acf(QStringLiteral("900002"), QStringLiteral("Half Done"), 1026, 0));
        writeAll(root + QStringLiteral("/steamapps/appmanifest_900009.acf"), "\"AppState\"\n{\n\t\"appid\" \"900009\"\n"); // broken
        writeAll(lib2 + QStringLiteral("/steamapps/appmanifest_900003.acf"), acf(QStringLiteral("900003"), QStringLiteral("Desk Game"), 6, 1700000000));
        writeAll(lib2 + QStringLiteral("/steamapps/appmanifest_900004.acf"), acf(QStringLiteral("900004"), QStringLiteral("Fps Overlay"), 4, 0));
        writeAll(lib2 + QStringLiteral("/steamapps/appmanifest_900005.acf"), acf(QStringLiteral("900005"), QStringLiteral("alpha never played"), 4, 0));
        writeAll(root + QStringLiteral("/config/steamapps.vrmanifest"), kVrManifest);
        const QString cache = root + QStringLiteral("/appcache/librarycache");
        writeAll(cache + QStringLiteral("/900001/0123abcd/library_capsule.jpg"), "jpg");  // newer: hashed sub-folder
        writeAll(cache + QStringLiteral("/900001/0123abcd/library_header.jpg"), "jpg");
        writeAll(cache + QStringLiteral("/900003/header.jpg"), "jpg");                     // header only
        writeAll(cache + QStringLiteral("/900004_library_600x900.jpg"), "jpg");             // older flat layout
    }
};

// A SteamVR status as the link gives it.
SteamVrStatus connected(bool hmdTracked, int compositorPid) {
    SteamVrStatus s;
    s.state = VrLinkState::Connected;
    VrDevice h;
    h.index = 0;
    h.cls = VrClass::Hmd;
    h.poseValid = hmdTracked;
    h.trackingResult = hmdTracked ? vrtrack::RunningOk : vrtrack::RunningOutOfRange;
    s.devices.push_back(h);
    s.compositorPid = compositorPid;
    return s;
}

ReadyInputs readyIn() {
    ReadyInputs in;
    in.vrserverRunning = true;
    in.link = connected(true, 1234);
    in.monitorState = QStringLiteral("Ready");
    return in;
}

} // namespace

// ---------------------------------------------------------------- VDF

TEST(vdf_reads_nested_blocks_case_insensitive_keys_and_numbers) {
    const VdfDoc d = parseVdf(QString::fromUtf8(acf(QStringLiteral("620"), QStringLiteral("A Game"), 4, 1792285852)));
    CHECK(d.ok());
    const VdfNode* st = d.root.child(u"appstate");
    CHECK(st && st->block);
    CHECK(st->str(u"APPID") == QStringLiteral("620"));
    CHECK(st->str(u"name") == QStringLiteral("A Game"));
    CHECK(st->num(u"lastplayed") == 1792285852);
    CHECK(st->num(u"missing", -7) == -7);
    CHECK(st->num(u"name", -1) == -1); // not a number
    const VdfNode* dep = st->child(u"InstalledDepots");
    CHECK(dep && dep->block && dep->child(u"620") && dep->child(u"620")->str(u"manifest") == QStringLiteral("1"));
    CHECK(st->str(u"InstalledDepots", QStringLiteral("x")) == QStringLiteral("x")); // a block is no string
}

TEST(vdf_escapes_comments_bare_words_and_condition_tags) {
    const VdfDoc d = parseVdf(uR"(// a comment line
#include "other.vdf"
"root" {
  "path"   "C:\\Games\\Steam"   // windows path with doubled backslashes
  "quote"  "say \"hi\"\tok"
  bare     word
  "cond"   "1" [$WIN32]
  "odd"    "a\qb"
  "empty"  ""
}
)");
    CHECK(d.ok());
    const VdfNode* r = d.root.child(u"root");
    CHECK(r != nullptr);
    if (!r) {
        return;
    }
    CHECK(r->str(u"path") == QStringLiteral("C:\\Games\\Steam"));
    CHECK(r->str(u"quote") == QStringLiteral("say \"hi\"\tok"));
    CHECK(r->str(u"bare") == QStringLiteral("word"));
    CHECK(r->str(u"cond") == QStringLiteral("1"));
    CHECK(r->str(u"odd") == QStringLiteral("a\\qb"));
    CHECK(r->child(u"empty") && r->str(u"empty", QStringLiteral("x")).isEmpty());
    CHECK(r->children.size() == 6);
}

TEST(vdf_broken_input_gives_what_was_read_and_an_error) {
    const VdfDoc open = parseVdf(u"\"a\" { \"b\" \"1\" ");
    CHECK(!open.ok());
    CHECK(open.root.child(u"a") && open.root.child(u"a")->str(u"b") == QStringLiteral("1"));
    CHECK(!parseVdf(u"\"a\" \"unclosed").ok());
    CHECK(!parseVdf(u"}").ok());
    CHECK(!parseVdf(u"{ \"a\" \"b\" }").ok());
    CHECK(!parseVdf(u"\"key-without-value\"").ok());
    CHECK(parseVdf(u"").ok());
    CHECK(parseVdf(u"\xFEFF\"bom\" \"1\"").root.str(u"bom") == QStringLiteral("1"));
    QString deep;
    for (int i = 0; i < 200; i++) {
        deep += QStringLiteral("\"k\" { ");
    }
    CHECK(!parseVdf(deep).ok()); // stops at the depth limit instead of recursing on
    CHECK(!readVdfFile(QStringLiteral("/nonexistent/file.vdf")).ok());
}

// ---------------------------------------------------------------- Steam library

TEST(steam_library_reads_every_library_and_skips_tools_and_unfinished_installs) {
    FakeSteam fs;
    const SteamLibrary lib = readSteamLibrary(fs.root);
    CHECK(lib.found);
    CHECK(lib.vrKnown);
    CHECK(lib.libraries.size() == 2); // the root and lib2; the missing folder is skipped
    QStringList ids;
    for (const SteamGame& g : lib.games) {
        ids << g.appId;
    }
    // last played first (900001 has the later SteamVR time), then by name (900003 and 900004 tie), never-played last
    CHECK(ids == (QStringList{QStringLiteral("900001"), QStringLiteral("900003"), QStringLiteral("900004"),
                              QStringLiteral("900005")}));
    CHECK(lib.errors.size() == 1); // the broken manifest, in plain words
    const SteamGame& sky = lib.games[0];
    CHECK(sky.name == QStringLiteral("Sky Racer"));
    CHECK(sky.vr);
    CHECK(sky.lastPlayed == 1800000500); // the later of the manifest and SteamVR's list
    CHECK(sky.installDir == fs.root + QStringLiteral("/steamapps/common/Sky Racer"));
    CHECK(sky.cover.endsWith(QStringLiteral("/900001/0123abcd/library_capsule.jpg")));
    CHECK(sky.coverPortrait);
    const SteamGame& overlay = lib.games[2];
    CHECK(overlay.vr); // VR overlays count as VR
    CHECK(overlay.cover.endsWith(QStringLiteral("/900004_library_600x900.jpg")) && overlay.coverPortrait);
    const SteamGame& desk = lib.games[1];
    CHECK(!desk.vr);
    CHECK(desk.cover.endsWith(QStringLiteral("/900003/header.jpg")) && !desk.coverPortrait);
    CHECK(lib.games[3].cover.isEmpty());
}

TEST(steam_library_without_vr_list_or_steam) {
    FakeSteam fs;
    QFile::remove(fs.root + QStringLiteral("/config/steamapps.vrmanifest"));
    const SteamLibrary lib = readSteamLibrary(fs.root);
    CHECK(lib.found && !lib.vrKnown);
    for (const SteamGame& g : lib.games) {
        CHECK(!g.vr);
    }
    QTemporaryDir empty;
    CHECK(!readSteamLibrary(empty.path()).found);
    // the old config/libraryfolders.vdf and the old "1" "/path" form
    QTemporaryDir t;
    const QString root = t.path() + QStringLiteral("/S");
    const QString other = t.path() + QStringLiteral("/O");
    QDir().mkpath(root + QStringLiteral("/steamapps"));
    writeAll(root + QStringLiteral("/config/libraryfolders.vdf"),
             QStringLiteral("\"LibraryFolders\" { \"TimeNextStatsReport\" \"1\" \"1\" \"%1\" }").arg(other).toUtf8());
    writeAll(other + QStringLiteral("/steamapps/appmanifest_900007.acf"), acf(QStringLiteral("900007"), QStringLiteral("Old"), 4, 0));
    const SteamLibrary old = readSteamLibrary(root);
    CHECK(old.games.size() == 1 && old.games[0].appId == QStringLiteral("900007"));
}

TEST(steam_tools_are_never_games) {
    CHECK(isSteamTool(QStringLiteral("250820"), QStringLiteral("SteamVR")));
    CHECK(isSteamTool(QStringLiteral("1628350"), QStringLiteral("x")));
    CHECK(isSteamTool(QStringLiteral("1"), QStringLiteral("Proton 12.0")));
    CHECK(isSteamTool(QStringLiteral("1"), QStringLiteral("Steam Linux Runtime 9.0 (future)")));
    CHECK(isSteamTool(QStringLiteral("1"), QStringLiteral("Steamworks Common Redistributables")));
    CHECK(isSteamTool(QStringLiteral("1"), QStringLiteral("Some Redistributable Pack")));
    CHECK(!isSteamTool(QStringLiteral("620980"), QStringLiteral("A VR Game")));
    CHECK(!isSteamTool(QStringLiteral("1"), QStringLiteral("SteamVR Home Arcade"))); // only SteamVR itself
}

TEST(steam_roots_drop_symlinked_duplicates) {
    QTemporaryDir t;
    const QString home = t.path();
    QDir().mkpath(home + QStringLiteral("/.local/share/Steam/steamapps"));
    QDir().mkpath(home + QStringLiteral("/.steam"));
    QFile::link(home + QStringLiteral("/.local/share/Steam"), home + QStringLiteral("/.steam/steam"));
    QFile::link(home + QStringLiteral("/.local/share/Steam"), home + QStringLiteral("/.steam/root"));
    const QStringList r = steamRoots(home, home + QStringLiteral("/.local/share"));
    CHECK(r.size() == 1);
    CHECK(r.value(0) == QFileInfo(home + QStringLiteral("/.local/share/Steam")).canonicalFilePath());
    QTemporaryDir none;
    CHECK(steamRoots(none.path(), none.path() + QStringLiteral("/data")).isEmpty());
}

TEST(steam_running_games_come_from_the_reaper_command_line) {
    QTemporaryDir t;
    const auto proc = [&t](const QString& pid, const QByteArray& cmdline) {
        writeAll(t.path() + QLatin1Char('/') + pid + QStringLiteral("/cmdline"), cmdline);
    };
    const auto args = [](std::initializer_list<const char*> l) {
        QByteArray b;
        for (const char* a : l) {
            b += a;
            b += '\0';
        }
        return b;
    };
    proc(QStringLiteral("100"), args({"/s/reaper", "SteamLaunch", "AppId=900001", "--", "/s/wrapper"}));
    proc(QStringLiteral("101"), args({"/usr/bin/game", "AppId=900002"})); // no SteamLaunch: not counted
    proc(QStringLiteral("102"), args({"/s/reaper", "SteamLaunch", "--", "x", "AppId=900003"})); // after "--": the game's
    proc(QStringLiteral("103"), args({"/s/reaper", "SteamLaunch", "AppId=12x"}));
    proc(QStringLiteral("self"), args({"/s/reaper", "SteamLaunch", "AppId=900004"})); // not a pid folder
    const QSet<QString> r = runningSteamApps(t.path());
    CHECK(r == QSet<QString>{QStringLiteral("900001")});
    CHECK(steamRunArgs(QStringLiteral("620980")) == QStringList{QStringLiteral("steam://rungameid/620980")});
}

TEST(steam_vr_manifest_broken_or_missing) {
    QTemporaryDir t;
    CHECK(!readVrManifest(t.path() + QStringLiteral("/none")).ok);
    writeAll(t.path() + QStringLiteral("/bad"), "{ not json");
    CHECK(!readVrManifest(t.path() + QStringLiteral("/bad")).ok);
    writeAll(t.path() + QStringLiteral("/good"), kVrManifest);
    const VrManifest m = readVrManifest(t.path() + QStringLiteral("/good"));
    CHECK(m.ok && m.apps.size() == 2);
    CHECK(m.apps.value(QStringLiteral("900001")) == 1800000500);
    CHECK(m.apps.value(QStringLiteral("900004")) == 1700000000);
}

// ---------------------------------------------------------------- apps.toml

TEST(apps_toml_round_trip_keeps_unknown_keys_comments_and_tables) {
    const QString text = QStringLiteral(
        "# my apps\n"
        "version = 2\n"
        "\n"
        "[[app]]\n"
        "id = \"sim\"\n"
        "name = \"Flight \\\"Sim\\\"\"\n"
        "exec = '~/tools/sim.sh'\n"
        "args = [\n"
        "  \"-xr\",  # the VR switch\n"
        "  \"--log\",\n"
        "]\n"
        "workdir = \"/games/sim\"\n"
        "env = [\"A=1\", \"B=two words\"]\n"
        "quad_views = false  # off for this one\n"
        "future_key = { nested = 1 }\n"
        "# a note\n"
        "last_played = 1_800_000_000\n"
        "[app.extra]\n"
        "x = 1\n"
        "\n"
        "[ui]\n"
        "zoom = 1.5\n"
        "\n"
        "[[app]]\n"
        "name = \"Second\"\n"
        "exec = \"/bin/second\"\n"
        "notes = \"\"\"multi\"\"\"\n");
    QStringList warnings;
    AppsDoc d = AppsDoc::parse(text, &warnings);
    const std::vector<OwnApp> apps = d.apps();
    CHECK(apps.size() == 2);
    if (apps.size() != 2) {
        return;
    }
    const OwnApp& a = apps[0];
    CHECK(a.id == QStringLiteral("sim"));
    CHECK(a.name == QStringLiteral("Flight \"Sim\""));
    CHECK(a.exec == QStringLiteral("~/tools/sim.sh"));
    CHECK(a.args == (QStringList{QStringLiteral("-xr"), QStringLiteral("--log")}));
    CHECK(a.workdir == QStringLiteral("/games/sim"));
    CHECK(a.env == (QStringList{QStringLiteral("A=1"), QStringLiteral("B=two words")}));
    CHECK(!a.quadViews);
    CHECK(a.lastPlayed == 1800000000);
    CHECK(a.extra.contains(QStringLiteral("future_key = { nested = 1 }")));
    CHECK(a.extra.contains(QStringLiteral("# a note")));
    CHECK(a.extra.contains(QStringLiteral("[app.extra]")) && a.extra.contains(QStringLiteral("x = 1")));
    CHECK(apps[1].id == QStringLiteral("second")); // from the name
    CHECK(apps[1].extra.contains(QStringLiteral("notes = \"\"\"multi\"\"\"")));
    CHECK(warnings.isEmpty());

    // written back: every unknown line is still there, and reading it again gives the same apps
    const QString out = d.text();
    for (const char* keep : {"# my apps", "version = 2", "future_key = { nested = 1 }", "[app.extra]", "[ui]",
                             "zoom = 1.5", "notes = \"\"\"multi\"\"\""}) {
        CHECK(out.contains(QString::fromUtf8(keep)));
    }
    const std::vector<OwnApp> again = AppsDoc::parse(out).apps();
    CHECK(again.size() == 2);
    if (again.size() == 2) {
        CHECK(again[0].name == a.name && again[0].args == a.args && again[0].env == a.env && !again[0].quadViews);
        CHECK(again[0].lastPlayed == a.lastPlayed && again[0].extra == a.extra);
        CHECK(again[1].id == QStringLiteral("second"));
    }
    CHECK(AppsDoc::parse(out).text() == out); // stable
    // [ui] stays between the two apps
    CHECK(out.indexOf(QStringLiteral("[ui]")) > out.indexOf(QStringLiteral("id = \"sim\"")));
    CHECK(out.indexOf(QStringLiteral("[ui]")) < out.indexOf(QStringLiteral("name = \"Second\"")));
}

TEST(apps_toml_upsert_remove_and_unique_ids) {
    AppsDoc d;
    OwnApp a;
    a.name = QStringLiteral("Falcon BMS");
    a.exec = QStringLiteral("/x/run.sh");
    CHECK(d.upsert(a) == QStringLiteral("falcon-bms"));
    CHECK(d.upsert(a) == QStringLiteral("falcon-bms-2")); // a second app with the same name
    OwnApp edit = *d.find(QStringLiteral("falcon-bms"));
    edit.args = {QStringLiteral("-xr")};
    edit.extra = {QStringLiteral("kept = true")};
    CHECK(d.upsert(edit) == QStringLiteral("falcon-bms"));
    CHECK(d.apps().size() == 2);
    CHECK(d.find(QStringLiteral("falcon-bms"))->args == QStringList{QStringLiteral("-xr")});
    CHECK(d.text().contains(QStringLiteral("kept = true")));
    CHECK(d.remove(QStringLiteral("falcon-bms-2")));
    CHECK(!d.remove(QStringLiteral("falcon-bms-2")));
    CHECK(d.apps().size() == 1);
    // duplicate ids in a hand-edited file are made unique
    const AppsDoc dup = AppsDoc::parse(u"[[app]]\nid = \"x\"\nname = \"A\"\n[[app]]\nid = \"x\"\nname = \"B\"\n");
    CHECK(dup.apps().size() == 2 && dup.apps()[1].id == QStringLiteral("x-2"));
    CHECK(appSlug(QStringLiteral("  Ünïcode & Co. 2 ")) == QStringLiteral("n-code-co-2"));
    CHECK(appSlug(QStringLiteral("!!!")) == QStringLiteral("app"));
}

TEST(apps_toml_strings_and_bad_lines) {
    CHECK(tomlQuote(QStringLiteral("a\"b\\c\nd\x01")) == QStringLiteral("\"a\\\"b\\\\c\\nd\\u0001\""));
    CHECK(tomlStringArray({QStringLiteral("x"), QStringLiteral("y z")}) == QStringLiteral("[\"x\", \"y z\"]"));
    QStringList w;
    const AppsDoc d = AppsDoc::parse(u"[[app]]\nname = \"\\u00e9t\\u00E9\"\nthis is not toml\nexec = 5\n", &w);
    CHECK(d.apps().size() == 1);
    CHECK(d.apps()[0].name == QStringLiteral("été"));
    CHECK(d.apps()[0].exec.isEmpty());                                      // wrong type: kept as a line
    CHECK(d.apps()[0].extra.contains(QStringLiteral("exec = 5")));
    CHECK(d.apps()[0].extra.contains(QStringLiteral("this is not toml")));
    CHECK(w.size() == 1);
    CHECK(AppsDoc::parse(u"").text().isEmpty());
}

TEST(apps_store_writes_atomically_and_reads_back) {
    QTemporaryDir t;
    const QString path = t.path() + QStringLiteral("/cfg/pibridge/apps.toml");
    AppsStore s(path);
    CHECK(s.load().isEmpty());
    CHECK(s.doc().apps().empty());
    OwnApp a;
    a.name = QStringLiteral("Viewer");
    a.exec = QStringLiteral("/bin/viewer");
    a.env = {QStringLiteral("K=v")};
    s.doc().upsert(a);
    QString err;
    CHECK(s.save(&err));
    CHECK(QFileInfo::exists(path));
    CHECK(QDir(QFileInfo(path).absolutePath()).entryList(QDir::Files).size() == 1); // no temp file left
    AppsStore r(path);
    r.load();
    CHECK(r.doc().apps().size() == 1 && r.doc().apps()[0].env == QStringList{QStringLiteral("K=v")});
    // a folder that cannot be written: false and a message
    AppsStore bad(QStringLiteral("/proc/pibridge-test/apps.toml"));
    CHECK(!bad.save(&err) && !err.isEmpty());
}

TEST(apps_paths_follow_xdg) {
    const QByteArray oldC = qgetenv("XDG_CONFIG_HOME"), oldS = qgetenv("XDG_STATE_HOME");
    qputenv("XDG_CONFIG_HOME", "/tmp/cfg-test");
    qputenv("XDG_STATE_HOME", "/tmp/state-test");
    CHECK(appsConfigPath() == QStringLiteral("/tmp/cfg-test/pibridge/apps.toml"));
    CHECK(appsLogDir() == QStringLiteral("/tmp/state-test/pibridge/apps"));
    qputenv("XDG_CONFIG_HOME", oldC);
    qputenv("XDG_STATE_HOME", oldS);
}

TEST(own_app_validation_and_profile_name) {
    OwnApp a;
    CHECK(validateOwnApp(a).size() == 2);
    a.name = QStringLiteral("X");
    a.exec = QStringLiteral("/opt/game/Falcon BMS.exe");
    a.env = {QStringLiteral("OK=1"), QStringLiteral("=bad"), QStringLiteral("1X=no")};
    const QStringList p = validateOwnApp(a);
    CHECK(p.size() == 2 && p[0].contains(QStringLiteral("line 2")));
    CHECK(qvProfileName(a) == QStringLiteral("Falcon BMS"));
    a.qvProfile = QStringLiteral(" Falcon ");
    CHECK(qvProfileName(a) == QStringLiteral("Falcon"));
}

TEST(args_line_round_trip_and_last_played_words) {
    const QStringList args{QStringLiteral("-xr"), QStringLiteral("two words"), QStringLiteral("say \"hi\""), QString()};
    const QString line = joinArgs(args);
    CHECK(line == QStringLiteral("-xr \"two words\" \"say \\\"hi\\\"\" \"\""));
    CHECK(splitArgs(line) == args);
    CHECK(splitArgs(QStringLiteral("  -a   -b ")) == (QStringList{QStringLiteral("-a"), QStringLiteral("-b")}));
    CHECK(splitArgs(QStringLiteral("'C:\\Games' a\\ b")) == (QStringList{QStringLiteral("C:\\Games"), QStringLiteral("a b")}));
    const QDateTime now(QDate(2026, 10, 19), QTime(15, 0), QTimeZone::UTC);
    const auto at = [](int y, int m, int d, int h, int min) {
        return QDateTime(QDate(y, m, d), QTime(h, min), QTimeZone::UTC).toSecsSinceEpoch();
    };
    CHECK(lastPlayedWords(0, now) == QStringLiteral("Never played"));
    CHECK(lastPlayedWords(at(2026, 10, 19, 9, 5), now) == QStringLiteral("Played today, 09:05"));
    CHECK(lastPlayedWords(at(2026, 10, 18, 23, 59), now) == QStringLiteral("Played yesterday"));
    CHECK(lastPlayedWords(at(2026, 10, 14, 12, 0), now) == QStringLiteral("Played 5 days ago"));
    CHECK(lastPlayedWords(at(2026, 9, 2, 12, 0), now) == QStringLiteral("Played 2 Sep 2026"));
}

// ---------------------------------------------------------------- launch spec and logs

TEST(own_app_spec_environment_quad_views_and_log_names) {
    OwnApp a;
    a.id = QStringLiteral("bms");
    a.exec = QStringLiteral("~/tools/bms-log.sh");
    a.args = {QStringLiteral("-xr")};
    a.env = {QStringLiteral("A=1"), QStringLiteral("  ")};
    a.quadViews = false;
    const QDateTime t = QDateTime::fromSecsSinceEpoch(1800000000, QTimeZone::UTC);
    LaunchSpec s = ownAppSpec(a, QStringLiteral("/logs"), t, QStringLiteral("/home/u"));
    CHECK(s.program == QStringLiteral("/home/u/tools/bms-log.sh"));
    CHECK(s.args == QStringList{QStringLiteral("-xr")});
    CHECK(s.workdir == QStringLiteral("/home/u/tools")); // none given: the program's folder
    CHECK(s.env == (QStringList{QStringLiteral("A=1"), QStringLiteral("DISABLE_XR_APILAYER_ARTSCOUT_QUAD_VIEWS_VULKAN=1")}));
    CHECK(s.logPath == QStringLiteral("/logs/bms-20270115-080000Z.log"));
    CHECK(s.logLink == QStringLiteral("/logs/bms-latest.log"));
    a.quadViews = true;
    a.workdir = QStringLiteral("/games/BMS");
    s = ownAppSpec(a, QStringLiteral("/logs"), t, QStringLiteral("/home/u"));
    CHECK(s.env == QStringList{QStringLiteral("A=1")});
    CHECK(s.workdir == QStringLiteral("/games/BMS"));
    CHECK(expandHome(QStringLiteral("~"), QStringLiteral("/h")) == QStringLiteral("/h"));
    CHECK(expandHome(QStringLiteral("~user/x"), QStringLiteral("/h")) == QStringLiteral("~user/x"));
}

TEST(prepare_log_creates_the_file_and_moves_the_latest_link) {
    QTemporaryDir t;
    LaunchSpec s;
    s.program = QStringLiteral("/bin/x");
    s.logPath = t.path() + QStringLiteral("/apps/x-1.log");
    s.logLink = t.path() + QStringLiteral("/apps/x-latest.log");
    QString err;
    CHECK(prepareLog(s, &err));
    CHECK(QFileInfo(s.logLink).isSymLink());
    CHECK(QFileInfo(s.logLink).symLinkTarget() == QFileInfo(s.logPath).absoluteFilePath());
    s.logPath = t.path() + QStringLiteral("/apps/x-2.log");
    CHECK(prepareLog(s, &err));
    CHECK(QFileInfo(s.logLink).symLinkTarget() == QFileInfo(s.logPath).absoluteFilePath());
    CHECK(readAll(s.logPath).startsWith("# PiBridge started: /bin/x"));
    // a real file in place of the link is left alone
    QFile::remove(s.logLink);
    writeAll(s.logLink, "mine");
    CHECK(prepareLog(s, &err));
    CHECK(readAll(s.logLink) == "mine");
}

// ---------------------------------------------------------------- readiness

TEST(ready_steps_in_order_with_plain_reasons) {
    ReadyInputs in;
    in.installed = false;
    CHECK(evaluateReady(in).step == ReadyStep::NotInstalled);
    in.installed = true;
    CHECK(evaluateReady(in).step == ReadyStep::NoServer);
    in.vrserverRunning = true;
    in.link.state = VrLinkState::Waiting;
    in.link.error = QStringLiteral("SteamVR is not ready yet");
    Readiness r = evaluateReady(in);
    CHECK(r.step == ReadyStep::NoClients);
    CHECK(readyFailureReason(r) == QStringLiteral("SteamVR started but does not answer (SteamVR is not ready yet)"));
    in.link = connected(true, 0);
    in.link.devices.clear();
    CHECK(readyFailureReason(evaluateReady(in)) == QStringLiteral("SteamVR does not see the headset"));
    in.link = connected(false, 0);
    r = evaluateReady(in);
    CHECK(r.step == ReadyStep::HeadsetNotTracked && r.detail == QStringLiteral("Out of range"));
    in.link = connected(true, 0);
    CHECK(readyFailureReason(evaluateReady(in)) == QStringLiteral("the SteamVR compositor did not start"));
    // IVRApplications not available: the /proc check decides
    in.link.compositorPid = -1;
    CHECK(evaluateReady(in).step == ReadyStep::NoCompositor);
    in.compositorProcess = true;
    CHECK(evaluateReady(in).step == ReadyStep::Settling);
    // a pid from OpenVR wins over the /proc check
    in.link.compositorPid = 0;
    CHECK(evaluateReady(in).step == ReadyStep::NoCompositor);
    in.link.compositorPid = 77;
    in.compositorProcess = false;
    for (const char* st : {"Startup", "Connecting", "NotReady", "Restart"}) {
        in.monitorState = QString::fromLatin1(st);
        CHECK(evaluateReady(in).step == ReadyStep::MonitorNotReady);
    }
    for (const char* st : {"Ready", "Standby", "Shutdown", "Off", ""}) { // a stale log never blocks
        in.monitorState = QString::fromLatin1(st);
        CHECK(evaluateReady(in).step == ReadyStep::Settling);
    }
    in.link.state = VrLinkState::NotInstalled;
    CHECK(readyFailureReason(evaluateReady(in)) == QStringLiteral("SteamVR is not installed"));
}

TEST(ready_monitor_log_tail) {
    const QString log = QStringLiteral(
        "Mon Oct 19 2026 02:44:53.693 [Info] - [System] Transition from 'SteamVRSystemState_Startup' to "
        "'SteamVRSystemState_Connecting'.\n"
        "Mon Oct 19 2026 02:45:02.828 [Info] - [System] Transition from 'SteamVRSystemState_NotReady' to "
        "'SteamVRSystemState_Ready'.\n"
        "Mon Oct 19 2026 02:45:03.000 [Info] - something else\n");
    CHECK(lastSystemState(log) == QStringLiteral("Ready"));
    CHECK(lastSystemState(u"no states here").isEmpty());
    CHECK(lastSystemState(u"to 'SteamVRSystemState_Broken").isEmpty());
    QTemporaryDir t;
    QByteArray big(100 * 1024, 'x');
    big += "\nUnknown Transition from 'SteamVRSystemState_Ready' to 'SteamVRSystemState_Standby'.\n";
    writeAll(t.path() + QStringLiteral("/vrmonitor.txt"), big);
    CHECK(lastSystemStateOfFile(t.path() + QStringLiteral("/vrmonitor.txt")) == QStringLiteral("Standby"));
    CHECK(lastSystemStateOfFile(t.path() + QStringLiteral("/none")).isEmpty());
}

TEST(ready_tracker_settles_and_restarts_the_settle_time_after_a_break) {
    qint64 now = 1000;
    ReadyTracker tr([&now] { return now; });
    CHECK(tr.update(readyIn()).step == ReadyStep::Settling);
    now += 1499;
    CHECK(tr.recheck().step == ReadyStep::Settling);
    now += 1;
    CHECK(tr.recheck().ready());
    ReadyInputs off = readyIn();
    off.link = connected(false, 1234); // headset lost
    CHECK(tr.update(off).step == ReadyStep::HeadsetNotTracked);
    CHECK(tr.update(readyIn()).step == ReadyStep::Settling); // the settle time starts again
    now += 1000;
    CHECK(!tr.recheck().ready());
    now += 500;
    CHECK(tr.recheck().ready());
}

TEST(ready_wait_succeeds_after_settle_times_out_with_the_step_and_fails_at_once_without_steamvr) {
    qint64 now = 0;
    SteamVrReady ready([&now] { return now; });
    int calls = 0;
    bool ok = false;
    QString reason;
    const auto cb = [&](bool r, const QString& why) {
        calls++;
        ok = r;
        reason = why;
    };
    ready.waitReady(60000, cb);
    CHECK(calls == 0 && ready.waiting() == 1);
    ReadyInputs in = readyIn();
    ready.setInputs(in);
    CHECK(calls == 0); // settling
    now += 1500;
    ready.poll();
    CHECK(calls == 1 && ok && reason.isEmpty());
    CHECK(ready.waiting() == 0);
    // already ready: at once
    ready.waitReady(60000, cb);
    CHECK(calls == 2 && ok);

    // timeout: the reason of the step that did not hold
    in.link = connected(true, 0);
    ready.setInputs(in);
    ready.waitReady(60000, cb);
    now += 59999;
    ready.poll();
    CHECK(calls == 2);
    now += 1;
    ready.poll();
    CHECK(calls == 3 && !ok && reason == QStringLiteral("the SteamVR compositor did not start"));

    // not installed: at once
    in.installed = false;
    ready.setInputs(in);
    ready.waitReady(60000, cb);
    CHECK(calls == 4 && !ok && reason == QStringLiteral("SteamVR is not installed"));

    // cancel
    in.installed = true;
    ready.setInputs(in);
    ready.waitReady(60000, cb);
    ready.cancelAll(QStringLiteral("stopped"));
    CHECK(calls == 5 && !ok && reason == QStringLiteral("stopped"));
}

TEST(steamvr_link_reports_the_compositor_pid) {
    FakeSteamVrSource src;
    src.compositor = 4321;
    SteamVrLink link(src, [] { return qint64(0); });
    CHECK(link.tick(true).compositorPid == 4321);
    src.compositor = 0;
    CHECK(link.tick(true).compositorPid == 0);
    CHECK(link.tick(false).compositorPid == -1); // not connected: unknown
}

// ---------------------------------------------------------------- launcher

namespace {
struct LaunchRig {
    qint64 now = 0;
    FakeRunner runner;
    FakeAppProcesses procs;
    SteamVrReady ready{[this] { return now; }};
    bool vrserver = false;
    AppLauncher launcher{runner, procs, ready, [this] { return vrserver; }, QStringLiteral("/logs"),
                         [this] { return now; }};
    OwnApp app;
    LaunchRig() {
        app.id = QStringLiteral("sim");
        app.name = QStringLiteral("Sim");
        app.exec = QStringLiteral("/games/sim");
    }
    void makeReady() {
        vrserver = true;
        ready.setInputs(readyIn());
        now += kReadySettleMs;
        ready.poll();
    }
};
} // namespace

TEST(launcher_starts_at_once_when_steamvr_is_ready) {
    LaunchRig r;
    r.makeReady();
    QStringList played;
    QObject::connect(&r.launcher, &AppLauncher::played, [&played](const QString& id) { played << id; });
    r.launcher.launchOwn(r.app);
    CHECK(r.runner.calls.empty()); // SteamVR is not started again
    CHECK(r.procs.started.size() == 1 && r.procs.started[0].program == QStringLiteral("/games/sim"));
    AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
    CHECK(s.run == AppRun::Running && s.pid == 4000);
    CHECK(s.logPath.startsWith(QStringLiteral("/logs/sim-")));
    CHECK(played == QStringList{QStringLiteral("sim")});
    CHECK(appChip(s, true).text == QStringLiteral("RUNNING"));
    r.launcher.launchOwn(r.app); // already running: nothing
    CHECK(r.procs.started.size() == 1);
    CHECK(r.launcher.stopOwn(QStringLiteral("sim")));
    CHECK(r.procs.stopped == std::vector<qint64>{4000}); // that PID only
    r.procs.exit(4000, 3);
    s = r.launcher.ownState(QStringLiteral("sim"));
    CHECK(s.run == AppRun::Exited && s.exitCode == 3);
    CHECK(appRunWords(s) == QStringLiteral("Exited (code 3)"));
    CHECK(appChip(s, true).text == QStringLiteral("READY"));
    CHECK(appChip(s, false).text.isEmpty());
    CHECK(!r.launcher.stopOwn(QStringLiteral("sim")));
}

TEST(launcher_starts_steamvr_waits_until_fully_ready_then_starts) {
    LaunchRig r;
    r.launcher.launchOwn(r.app);
    CHECK(r.runner.calls.size() == 1);
    CHECK(r.runner.calls[0].detached && r.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/250820"));
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::StartingSteamVr);
    CHECK(appChip(r.launcher.ownState(QStringLiteral("sim")), false).text == QStringLiteral("STARTING STEAMVR"));
    // vrserver up, but only the server: not yet
    r.vrserver = true;
    ReadyInputs in;
    in.vrserverRunning = true;
    in.link.state = VrLinkState::Waiting;
    r.ready.setInputs(in);
    r.now += 5000;
    r.ready.poll();
    CHECK(r.procs.started.empty());
    in = readyIn();
    in.link.compositorPid = 0; // headset tracked, compositor not yet
    r.ready.setInputs(in);
    r.now += 5000;
    r.ready.poll();
    CHECK(r.procs.started.empty());
    r.ready.setInputs(readyIn());
    r.now += 1000;
    r.ready.poll();
    CHECK(r.procs.started.empty()); // settling
    r.now += 500;
    r.ready.poll();
    CHECK(r.procs.started.size() == 1);
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Running);
    // a second app while SteamVR already runs and is ready: no steam call
    OwnApp b = r.app;
    b.id = QStringLiteral("b");
    r.launcher.launchOwn(b);
    CHECK(r.runner.calls.size() == 1 && r.procs.started.size() == 2);
}

TEST(launcher_does_not_start_steamvr_twice_and_reports_the_timeout) {
    LaunchRig r;
    r.vrserver = true; // vrserver runs, SteamVR not ready
    r.ready.setInputs([] {
        ReadyInputs in = readyIn();
        in.link.devices.clear();
        return in;
    }());
    r.launcher.launchOwn(r.app);
    CHECK(r.runner.calls.empty());
    r.now += kReadyTimeoutMs;
    r.ready.poll();
    const AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
    CHECK(s.run == AppRun::Error);
    CHECK(s.message == QStringLiteral("SteamVR does not see the headset"));
    CHECK(appChip(s, false).text == QStringLiteral("ERROR"));
    CHECK(r.procs.started.empty());
}

TEST(launcher_stop_while_waiting_cancels_the_start) {
    LaunchRig r;
    r.launcher.launchOwn(r.app);
    CHECK(r.launcher.stopOwn(QStringLiteral("sim")));
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Idle);
    r.makeReady();
    CHECK(r.procs.started.empty()); // the pending start was dropped
}

TEST(launcher_errors_in_plain_words) {
    LaunchRig r;
    r.makeReady();
    r.procs.failWith = QStringLiteral("Could not start sim: No such file or directory");
    r.launcher.launchOwn(r.app);
    AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
    CHECK(s.run == AppRun::Error && appRunWords(s) == r.procs.failWith);
    r.procs.failWith.clear();
    r.launcher.launchOwn(r.app); // retry after an error is allowed
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Running);
    r.procs.exit(4000, 0, true);
    CHECK(appRunWords(r.launcher.ownState(QStringLiteral("sim"))) == QStringLiteral("Crashed"));
}

TEST(launcher_steam_games) {
    LaunchRig r;
    r.makeReady();
    CHECK(r.launcher.launchSteam(QStringLiteral("620980")));
    CHECK(r.runner.calls.size() == 1 && r.runner.calls[0].detached);
    CHECK(r.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/620980"));
    CHECK(r.launcher.steamState(QStringLiteral("620980"), {}).run == AppRun::Starting);
    CHECK(r.launcher.steamState(QStringLiteral("620980"), {QStringLiteral("620980")}).run == AppRun::Running);
    // once seen running, its end does not show STARTING again
    CHECK(r.launcher.steamState(QStringLiteral("620980"), {}).run == AppRun::Idle);
    r.launcher.launchSteam(QStringLiteral("1"));
    r.now += kSteamStartShowMs;
    CHECK(r.launcher.steamState(QStringLiteral("1"), {}).run == AppRun::Idle);
}

TEST(fake_data_has_no_real_paths) {
    for (const SteamGame& g : fakeSteamLibrary(1800000000).games) {
        CHECK(g.cover.isEmpty() && g.installDir.isEmpty());
    }
    const auto apps = fakeOwnApps(1800000000);
    CHECK(apps.size() == 2);
    for (const OwnApp& a : apps) {
        CHECK(a.exec.startsWith(QStringLiteral("/fake/")));
    }
}

TEST(launcher_vr_steam_game_waits_for_steamvr) {
    // an OpenXR game started before SteamVR is up does not find the runtime: SteamVR first, the game when ready
    LaunchRig r;
    CHECK(r.launcher.launchSteam(QStringLiteral("620980"), true));
    CHECK(r.runner.calls.size() == 1 && r.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/250820"));
    CHECK(r.launcher.steamState(QStringLiteral("620980"), {}).run == AppRun::StartingSteamVr);
    CHECK(r.launcher.launchSteam(QStringLiteral("620980"), true)); // a second Play while waiting starts nothing
    CHECK(r.runner.calls.size() == 1);
    r.makeReady();
    CHECK(r.runner.calls.size() == 2 && r.runner.calls[1].line() == QStringLiteral("steam steam://rungameid/620980"));
    CHECK(r.launcher.steamState(QStringLiteral("620980"), {}).run == AppRun::Starting);
    // cancelled while waiting: the game is never started
    LaunchRig c;
    CHECK(c.launcher.launchSteam(QStringLiteral("7"), true));
    CHECK(c.launcher.cancelSteam(QStringLiteral("7")));
    c.makeReady();
    CHECK(c.runner.calls.size() == 1);
    CHECK(c.launcher.steamState(QStringLiteral("7"), {}).run == AppRun::Idle);
    // a non-VR game starts at once
    LaunchRig f;
    CHECK(f.launcher.launchSteam(QStringLiteral("8"), false));
    CHECK(f.runner.calls.size() == 1 && f.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/8"));
}

// ---------------------------------------------------------------- "Monado directly"

namespace {
// The runtime as the launcher sees it: a target, Monado's readiness, and whether starting tracking works.
struct FakeRuntime : AppRuntime {
    AppTarget t = AppTarget::Monado;
    MonadoReadiness m;
    bool canStart = true;
    QString cannot = QStringLiteral("SteamVR is running. Close it first.");
    int starts = 0;
    bool viaTracking = false; // the inside-out mode: SteamVR comes with START TRACKING
    AppTarget target() const override { return t; }
    MonadoReadiness monado() const override { return m; }
    bool steamVrThroughTracking() const override { return viaTracking; }
    bool startTracking(QString* why) override {
        starts++;
        if (!canStart) {
            *why = cannot;
            return false;
        }
        set(RecState::Starting);
        return true;
    }
    void set(RecState st, bool socket = true, bool home = true) {
        MonadoInputs in;
        in.runtimeJson = QStringLiteral("/m/openxr_monado-dev.json");
        in.state = st;
        in.socket = socket;
        in.homeSettled = home;
        m = evaluateMonadoReady(in);
    }
};

bool hasEnv(const LaunchSpec& s, const QString& kv) {
    return s.env.contains(kv);
}
} // namespace

TEST(monado_readiness_steps_in_plain_words) {
    MonadoInputs in;
    CHECK(evaluateMonadoReady(in).step == MonadoStep::NoRuntimeFile);
    CHECK(evaluateMonadoReady(in).words.contains(QStringLiteral("openxr_monado-dev.json")));
    in.runtimeJson = QStringLiteral("/m/openxr_monado-dev.json");
    CHECK(evaluateMonadoReady(in).step == MonadoStep::Off);
    CHECK(evaluateMonadoReady(in).words == QStringLiteral("Tracking is off. Play starts it."));
    in.state = RecState::Starting;
    CHECK(evaluateMonadoReady(in).words == QStringLiteral("Starting tracking..."));
    in.state = RecState::RetryWait;
    CHECK(evaluateMonadoReady(in).step == MonadoStep::Starting);
    in.state = RecState::Recording;
    CHECK(evaluateMonadoReady(in).step == MonadoStep::NoSocket);
    CHECK(evaluateMonadoReady(in).words == QStringLiteral("Waiting for Monado..."));
    in.socket = true;
    in.homeSettled = false;
    CHECK(evaluateMonadoReady(in).step == MonadoStep::HomeStarting);
    in.homeSettled = true;
    const MonadoReadiness r = evaluateMonadoReady(in);
    CHECK(r.ready() && r.runtimeJson == QStringLiteral("/m/openxr_monado-dev.json"));
    in.state = RecState::Failed;
    in.message = QStringLiteral("Monado exited (code 1)");
    CHECK(evaluateMonadoReady(in).words == QStringLiteral("Tracking stopped: Monado exited (code 1)"));
    CHECK(steamBlockedWhy(AppTarget::SteamVr).isEmpty());
    CHECK(steamBlockedWhy(AppTarget::Monado) ==
          QStringLiteral("Steam games run through SteamVR; stop \"Monado directly\" on the Development page first."));
    CHECK(monadoSocketPath(QStringLiteral("/run/user/1")) == QStringLiteral("/run/user/1/monado_comp_ipc"));
}

TEST(monado_runtime_file_and_libmonado_lookup) {
    QTemporaryDir t;
    // a build tree: <build>/src/xrt/targets/service/monado-service, the file in <build>/
    const QString b = t.path() + QStringLiteral("/build");
    const QString svc = b + QStringLiteral("/src/xrt/targets/service/monado-service");
    writeAll(svc, "");
    CHECK(monadoRuntimeJsonFor(svc).isEmpty());
    CHECK(libmonadoFor(svc).isEmpty());
    writeAll(b + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so"), "");
    CHECK(libmonadoFor(svc) == b + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so"));
    writeAll(b + QStringLiteral("/openxr_monado-dev.json"),
             "{\"file_format_version\": \"1.0.0\", \"runtime\": {\"library_path\": \"x.so\", "
             "\"MND_libmonado_path\": \"src/xrt/targets/libmonado/libmonado.so.25\"}}");
    CHECK(monadoRuntimeJsonFor(svc) == b + QStringLiteral("/openxr_monado-dev.json"));
    CHECK(libmonadoFor(svc) == b + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so")); // .so.25 missing
    writeAll(b + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so.25"), "");
    CHECK(libmonadoFor(svc) == b + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so.25")); // the file's own
    // an installed one: <prefix>/bin/monado-service
    const QString p = t.path() + QStringLiteral("/prefix");
    writeAll(p + QStringLiteral("/bin/monado-service"), "");
    CHECK(monadoRuntimeJsonFor(p + QStringLiteral("/bin/monado-service")).isEmpty());
    writeAll(p + QStringLiteral("/share/openxr/1/openxr_monado.json"), "{}");
    writeAll(p + QStringLiteral("/lib/libmonado.so"), "");
    CHECK(monadoRuntimeJsonFor(p + QStringLiteral("/bin/monado-service")) ==
          p + QStringLiteral("/share/openxr/1/openxr_monado.json"));
    CHECK(libmonadoFor(p + QStringLiteral("/bin/monado-service")) == p + QStringLiteral("/lib/libmonado.so"));
    CHECK(monadoRuntimeJsonFor(QString()).isEmpty());
}

TEST(monado_mode_starts_own_apps_on_monado_and_never_steamvr) {
    LaunchRig r;
    FakeRuntime rt;
    rt.set(RecState::Recording);
    r.launcher.setRuntime(&rt);
    r.launcher.launchOwn(r.app);
    CHECK(r.runner.calls.empty()); // no SteamVR
    CHECK(r.procs.started.size() == 1);
    CHECK(hasEnv(r.procs.started[0], QStringLiteral("XR_RUNTIME_JSON=/m/openxr_monado-dev.json")));
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Running);
    CHECK(appChip(r.launcher.ownState(QStringLiteral("sim")), true).text == QStringLiteral("RUNNING"));
    CHECK(rt.starts == 0);
    // SteamVR mode: no XR_RUNTIME_JSON (the system's active runtime), as before
    LaunchRig s;
    FakeRuntime st;
    st.t = AppTarget::SteamVr;
    s.launcher.setRuntime(&st);
    s.makeReady();
    s.launcher.launchOwn(s.app);
    CHECK(s.procs.started.size() == 1);
    for (const QString& e : s.procs.started[0].env) {
        CHECK(!e.startsWith(QStringLiteral("XR_RUNTIME_JSON=")));
    }
}

TEST(monado_mode_play_starts_tracking_and_waits_for_monado) {
    LaunchRig r;
    FakeRuntime rt;
    rt.set(RecState::Idle);
    r.launcher.setRuntime(&rt);
    r.launcher.launchOwn(r.app);
    CHECK(rt.starts == 1); // Play started the Tracking session
    AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
    CHECK(s.run == AppRun::StartingTracking);
    CHECK(appChip(s, false).text == QStringLiteral("STARTING TRACKING"));
    CHECK(appRunWords(s) == QStringLiteral("Starting tracking..."));
    CHECK(r.launcher.waitingForMonado() == 1);
    r.launcher.launchOwn(r.app); // a second Play while waiting: nothing
    CHECK(rt.starts == 1);
    rt.set(RecState::Recording, false); // Monado up, its socket not yet
    r.now += 5000;
    r.launcher.pollMonado();
    CHECK(r.procs.started.empty());
    CHECK(appRunWords(r.launcher.ownState(QStringLiteral("sim"))) == QStringLiteral("Waiting for Monado..."));
    rt.set(RecState::Recording, true, false); // the home scene is opening its session
    r.launcher.pollMonado();
    CHECK(r.procs.started.empty());
    rt.set(RecState::Recording);
    r.launcher.pollMonado();
    CHECK(r.procs.started.size() == 1);
    CHECK(hasEnv(r.procs.started[0], QStringLiteral("XR_RUNTIME_JSON=/m/openxr_monado-dev.json")));
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Running);
    CHECK(r.launcher.waitingForMonado() == 0);
    CHECK(r.runner.calls.empty());
}

TEST(monado_mode_errors_cancel_and_timeout) {
    {
        LaunchRig r; // the session cannot start
        FakeRuntime rt;
        rt.set(RecState::Idle);
        rt.canStart = false;
        r.launcher.setRuntime(&rt);
        r.launcher.launchOwn(r.app);
        const AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
        CHECK(s.run == AppRun::Error);
        CHECK(s.message == QStringLiteral("Tracking could not start: SteamVR is running. Close it first."));
        CHECK(r.runner.calls.empty() && r.procs.started.empty());
    }
    {
        LaunchRig r; // no runtime file: the app could not find Monado
        FakeRuntime rt;
        rt.m = evaluateMonadoReady(MonadoInputs{});
        r.launcher.setRuntime(&rt);
        r.launcher.launchOwn(r.app);
        CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Error);
        CHECK(rt.starts == 0);
    }
    {
        LaunchRig r; // the session fails while the app waits
        FakeRuntime rt;
        rt.set(RecState::Starting);
        r.launcher.setRuntime(&rt);
        r.launcher.launchOwn(r.app);
        CHECK(rt.starts == 0); // already starting
        rt.set(RecState::Failed);
        r.launcher.pollMonado();
        CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Error);
        CHECK(r.launcher.ownState(QStringLiteral("sim")).message == QStringLiteral("Tracking stopped."));
        CHECK(r.procs.started.empty());
    }
    {
        LaunchRig r; // cancel while waiting
        FakeRuntime rt;
        rt.set(RecState::Starting);
        r.launcher.setRuntime(&rt);
        r.launcher.launchOwn(r.app);
        CHECK(r.launcher.stopOwn(QStringLiteral("sim")));
        CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Idle);
        rt.set(RecState::Recording);
        r.launcher.pollMonado();
        CHECK(r.procs.started.empty());
    }
    {
        LaunchRig r; // Monado never gets ready
        FakeRuntime rt;
        rt.set(RecState::Recording, false);
        r.launcher.setRuntime(&rt);
        r.launcher.launchOwn(r.app);
        r.now += kReadyTimeoutMs;
        r.launcher.pollMonado();
        const AppRunState s = r.launcher.ownState(QStringLiteral("sim"));
        CHECK(s.run == AppRun::Error);
        CHECK(s.message == QStringLiteral("Monado was not ready after 60 s: Waiting for Monado..."));
        CHECK(r.procs.started.empty() && r.runner.calls.empty());
    }
}

TEST(monado_mode_refuses_steam_games_without_starting_anything) {
    LaunchRig r;
    FakeRuntime rt;
    rt.set(RecState::Recording);
    r.launcher.setRuntime(&rt);
    CHECK(!r.launcher.launchSteam(QStringLiteral("620980"), true));
    CHECK(!r.launcher.launchSteam(QStringLiteral("8"), false));
    CHECK(r.runner.calls.empty());
    const AppRunState s = r.launcher.steamState(QStringLiteral("620980"), {});
    CHECK(s.run == AppRun::Error && s.message == steamBlockedWhy(AppTarget::Monado));
    // SteamVR mode: unchanged
    rt.t = AppTarget::SteamVr;
    CHECK(r.launcher.launchSteam(QStringLiteral("8"), false));
    CHECK(r.runner.calls.size() == 1 && r.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/8"));
}

TEST(inside_out_starts_steamvr_through_tracking_never_through_steam) {
    // own app: SteamVR is off, so START TRACKING (SteamVR + Monado) is started instead of steam://rungameid/250820
    LaunchRig r;
    FakeRuntime rt;
    rt.t = AppTarget::SteamVr;
    rt.viaTracking = true;
    r.launcher.setRuntime(&rt);
    r.launcher.launchOwn(r.app);
    CHECK(rt.starts == 1 && r.runner.calls.empty());
    CHECK(r.launcher.ownState(QStringLiteral("sim")).run == AppRun::StartingSteamVr);
    r.makeReady(); // SteamVR came up with the Monado driver
    CHECK(r.procs.started.size() == 1 && r.launcher.ownState(QStringLiteral("sim")).run == AppRun::Running);
    CHECK(r.runner.calls.empty());
    // a VR Steam game: the same, then the game through Steam once SteamVR is ready
    LaunchRig g;
    FakeRuntime gt;
    gt.t = AppTarget::SteamVr;
    gt.viaTracking = true;
    g.launcher.setRuntime(&gt);
    CHECK(g.launcher.launchSteam(QStringLiteral("620980"), true));
    CHECK(gt.starts == 1 && g.runner.calls.empty());
    g.makeReady();
    CHECK(g.runner.calls.size() == 1 && g.runner.calls[0].line() == QStringLiteral("steam steam://rungameid/620980"));
    // tracking cannot start: the reason, nothing else is started
    LaunchRig e;
    FakeRuntime et;
    et.t = AppTarget::SteamVr;
    et.viaTracking = true;
    et.canStart = false;
    e.launcher.setRuntime(&et);
    e.launcher.launchOwn(e.app);
    CHECK(e.launcher.ownState(QStringLiteral("sim")).run == AppRun::Error);
    CHECK(e.launcher.ownState(QStringLiteral("sim")).message ==
          QStringLiteral("Tracking could not start: SteamVR is running. Close it first."));
    CHECK(!e.launcher.launchSteam(QStringLiteral("9"), true) && e.runner.calls.empty());
}

PIBRIDGE_TEST_MAIN()
