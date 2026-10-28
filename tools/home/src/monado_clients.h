// SPDX-License-Identifier: MIT
// The Monado session's clients through libmonado (dlopen: the "MND_libmonado_path" of the runtime file
// XR_RUNTIME_JSON names): which app is the primary one, and blocking its inputs while the dashboard overlay is open
// (as SteamVR's dashboard keeps a game from seeing the clicks on it). Monado makes every overlay session focused, so the
// overlay and the game both get the controllers; the block is what keeps them apart.
#pragma once

#include <memory>
#include <optional>
#include <string>

namespace pibridge::home {

class MonadoClients {
public:
    MonadoClients();
    ~MonadoClients();
    MonadoClients(const MonadoClients&) = delete;
    MonadoClients& operator=(const MonadoClients&) = delete;

    bool ok() const;
    /// The primary (main) app: its client id and name; nullopt when none or libmonado is not there.
    std::optional<std::pair<unsigned, std::string>> primary();
    /// Block (true) or release the inputs of a client.
    bool blockInputs(unsigned clientId, bool block);
    /// The client's process id (libmonado 1.9, PiBridge's fork: mnd_root_get_client_pid); nullopt when unknown.
    std::optional<long long> pidOf(unsigned clientId);
    /// pidOf() can work (the libmonado has the call).
    bool knowsPids() const;

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};

/// The libmonado library a runtime file names ("MND_libmonado_path"), "" when none.
std::string libmonadoOf(const std::string& runtimeJson);

} // namespace pibridge::home
