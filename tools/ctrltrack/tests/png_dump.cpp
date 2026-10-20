// SPDX-License-Identifier: MIT
// Decoded pixels of every PNG in a directory (name order): name, size, channels and an FNV-1a hash of the pixels.
// The deflate bytes of zlib and miniz_oxide differ; the pixels must not. Uses libpibridge-lens's PNG decoder.
#include "pibridge/lens/png.h"

#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        return 2;
    }
    std::vector<std::string> names;
    if (DIR* d = ::opendir(argv[1])) {
        while (const dirent* e = ::readdir(d)) {
            const std::string n = e->d_name;
            if (n != "." && n != "..") {
                names.push_back(n);
            }
        }
        ::closedir(d);
    } else {
        std::printf("no directory %s\n", argv[1]);
        return 0;
    }
    std::sort(names.begin(), names.end());
    for (const std::string& n : names) {
        std::ifstream f(std::string(argv[1]) + "/" + n, std::ios::binary);
        const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
        const auto img = pibridge::lens::png::decode(bytes);
        if (!img) {
            std::printf("%s: %s\n", n.c_str(), img.error().c_str());
            continue;
        }
        std::uint64_t h = 0xcbf29ce484222325ULL;
        for (const std::uint8_t b : img->data) {
            h = (h ^ b) * 0x100000001b3ULL;
        }
        std::printf("%s %ux%u x%zu %016llx\n", n.c_str(), img->width, img->height, img->channels,
                    static_cast<unsigned long long>(h));
    }
    return 0;
}
