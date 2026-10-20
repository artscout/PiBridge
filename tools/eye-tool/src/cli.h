// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/cli.rs (MIT OR Apache-2.0): the offline commands of the `pimax-eye` tool.
#pragma once

#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace eyetool {

/// The argument list of one command: flags and values are taken out by name, then positionals, then finish() refuses
/// anything left (as the Rust `Args`). Values accept both `--name value` and `--name=value`.
class Args {
public:
    explicit Args(std::vector<std::string> items) : items_(std::move(items)) {}

    bool flag(const std::string& name);
    /// The value of `name`; an error text when the value is missing.
    std::optional<std::string> value(const std::string& name, std::string& error);
    /// A decimal number; an error text when it does not parse.
    template <class T>
    std::optional<T> number(const std::string& name, std::string& error);
    /// The first argument that does not start with `--`.
    std::optional<std::string> positional();
    /// Empty when nothing is left, else the error text.
    std::string finish() const;

private:
    std::vector<std::string> items_;
};

/// Run the tool; returns the process exit code. 0 ok, 1 usage, 2 file/capture error.
int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

} // namespace eyetool
