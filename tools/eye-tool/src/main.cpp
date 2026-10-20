// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/main.rs (MIT OR Apache-2.0).
#include "cli.h"

#include <iostream>

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    return eyetool::run(args, std::cout, std::cerr);
}
