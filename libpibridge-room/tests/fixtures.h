// SPDX-License-Identifier: MIT
// Test helpers: a temporary directory and synthetic SteamVR files (made-up numbers, no real room).
#pragma once

#include "pibridge/room/chap.h"
#include "pibridge/room/files.h"

#include <cstdlib>
#include <filesystem>
#include <string>

namespace fixtures {

struct TempDir {
    std::string path;
    TempDir() {
        std::string tmpl = (std::filesystem::temp_directory_path() / "pibridge-room-test-XXXXXX").string();
        path = ::mkdtemp(tmpl.data());
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    std::string sub(const std::string& s) const {
        const std::string p = path + "/" + s;
        std::filesystem::create_directories(p);
        return p;
    }
};

/// Two SteamVR universes (a 4 x 3 m room around the standing origin, and a 3 x 2 m one in another universe), an
/// unknown top-level key and unknown keys inside a universe.
inline std::string vrchap() {
    return R"({
   "jsonid" : "chaperone_info",
   "unknown_top" : { "keep" : [ 1, 2, 3 ] },
   "universes" : [
      {
         "collision_bounds" : [
            [ [ -2.0, 0.0, -1.5 ], [ -2.0, 2.43, -1.5 ], [ 2.0, 2.43, -1.5 ], [ 2.0, 0.0, -1.5 ] ],
            [ [ 2.0, 0.0, -1.5 ], [ 2.0, 2.43, -1.5 ], [ 2.0, 2.43, 1.5 ], [ 2.0, 0.0, 1.5 ] ],
            [ [ 2.0, 0.0, 1.5 ], [ 2.0, 2.43, 1.5 ], [ -2.0, 2.43, 1.5 ], [ -2.0, 0.0, 1.5 ] ],
            [ [ -2.0, 0.0, 1.5 ], [ -2.0, 2.43, 1.5 ], [ -2.0, 2.43, -1.5 ], [ -2.0, 0.0, -1.5 ] ]
         ],
         "play_area" : [ 3, 2 ],
         "seated" : { "translation" : [ 0.1, 1.2, 0.2 ], "yaw" : 0.5 },
         "standing" : { "translation" : [ 0.25, 0.01, -0.5 ], "yaw" : 0.3 },
         "time" : "Mon Oct 19 10:00:00 2026",
         "universeID" : "1111111111",
         "x_vendor_field" : "keep me"
      },
      {
         "collision_bounds" : [
            [ [ -1.5, 0.0, -1.0 ], [ -1.5, 2.43, -1.0 ], [ 1.5, 2.43, -1.0 ], [ 1.5, 0.0, -1.0 ] ],
            [ [ 1.5, 0.0, -1.0 ], [ 1.5, 2.43, -1.0 ], [ 1.5, 2.43, 1.0 ], [ 1.5, 0.0, 1.0 ] ],
            [ [ 1.5, 0.0, 1.0 ], [ 1.5, 2.43, 1.0 ], [ -1.5, 2.43, 1.0 ], [ -1.5, 0.0, 1.0 ] ],
            [ [ -1.5, 0.0, 1.0 ], [ -1.5, 2.43, 1.0 ], [ -1.5, 2.43, -1.0 ], [ -1.5, 0.0, -1.0 ] ]
         ],
         "play_area" : [ 2.5, 1.5 ],
         "seated" : { "translation" : [ 0, 1.1, 0 ], "yaw" : 0 },
         "standing" : { "translation" : [ 0, 0, 0 ], "yaw" : 0 },
         "time" : "Sun Oct 18 09:00:00 2026",
         "universeID" : "2"
      }
   ],
   "version" : 5
}
)";
}

inline void write(const std::string& path, const std::string& text) {
    (void)pibridge::room::writeFileAtomic(path, text);
}

} // namespace fixtures
