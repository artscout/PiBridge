// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-record/src/main.rs (MIT OR Apache-2.0).
// pibridge-record: records what the headset shows (docs/record.md).
#include "pibridge/record/cli.h"

int main(int argc, char** argv) { return pibridge::record::main(std::vector<std::string>(argv + 1, argv + argc)); }
