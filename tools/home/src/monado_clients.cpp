// SPDX-License-Identifier: MIT
#include "monado_clients.h"

#include <cstdint>
#include <cstdlib>
#include <dlfcn.h>
#include <fstream>
#include <sstream>

namespace pibridge::home {

namespace {

// the libmonado C API (targets/libmonado/monado.h), the few calls used
using root_t = void;
using fn_create = int (*)(root_t**);
using fn_destroy = void (*)(root_t**);
using fn_update = int (*)(root_t*);
using fn_count = int (*)(root_t*, std::uint32_t*);
using fn_id_at = int (*)(root_t*, std::uint32_t, std::uint32_t*);
using fn_name = int (*)(root_t*, std::uint32_t, const char**);
using fn_state = int (*)(root_t*, std::uint32_t, std::uint32_t*);
using fn_blocks = int (*)(root_t*, std::uint32_t, std::uint32_t);
using fn_pid = int (*)(root_t*, std::uint32_t, std::int64_t*);
constexpr std::uint32_t kPrimary = 1u << 0u;     // MND_CLIENT_PRIMARY_APP
constexpr std::uint32_t kOverlay = 1u << 4u;     // MND_CLIENT_SESSION_OVERLAY
constexpr std::uint32_t kBlockInputs = 1u << 2u; // MND_IO_BLOCK_INPUTS

} // namespace

std::string libmonadoOf(const std::string& runtimeJson) {
    std::ifstream f(runtimeJson);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string t = ss.str();
    const auto k = t.find("\"MND_libmonado_path\"");
    if (k == std::string::npos) {
        return {};
    }
    const auto a = t.find('"', t.find(':', k) + 1);
    const auto b = a == std::string::npos ? a : t.find('"', a + 1);
    return a == std::string::npos || b == std::string::npos ? std::string() : t.substr(a + 1, b - a - 1);
}

struct MonadoClients::Impl {
    void* lib = nullptr;
    root_t* root = nullptr;
    fn_create create = nullptr;
    fn_destroy destroy = nullptr;
    fn_update update = nullptr;
    fn_count count = nullptr;
    fn_id_at idAt = nullptr;
    fn_name name = nullptr;
    fn_state state = nullptr;
    fn_blocks blocks = nullptr;
    fn_pid pid = nullptr; // optional (PiBridge's fork only)
};

MonadoClients::MonadoClients() : d_(std::make_unique<Impl>()) {
    const char* rt = std::getenv("XR_RUNTIME_JSON");
    const std::string path = rt != nullptr ? libmonadoOf(rt) : std::string();
    if (path.empty() || (d_->lib = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL)) == nullptr) {
        return;
    }
    const auto sym = [this](const char* n) { return ::dlsym(d_->lib, n); };
    d_->create = reinterpret_cast<fn_create>(sym("mnd_root_create"));
    d_->destroy = reinterpret_cast<fn_destroy>(sym("mnd_root_destroy"));
    d_->update = reinterpret_cast<fn_update>(sym("mnd_root_update_client_list"));
    d_->count = reinterpret_cast<fn_count>(sym("mnd_root_get_number_clients"));
    d_->idAt = reinterpret_cast<fn_id_at>(sym("mnd_root_get_client_id_at_index"));
    d_->name = reinterpret_cast<fn_name>(sym("mnd_root_get_client_name"));
    d_->state = reinterpret_cast<fn_state>(sym("mnd_root_get_client_state"));
    d_->blocks = reinterpret_cast<fn_blocks>(sym("mnd_root_set_client_io_blocks"));
    d_->pid = reinterpret_cast<fn_pid>(sym("mnd_root_get_client_pid"));
    if (!d_->create || !d_->destroy || !d_->update || !d_->count || !d_->idAt || !d_->name || !d_->state || !d_->blocks ||
        d_->create(&d_->root) != 0) {
        d_->root = nullptr;
    }
}

MonadoClients::~MonadoClients() {
    if (d_->root != nullptr) {
        d_->destroy(&d_->root);
    }
    if (d_->lib != nullptr) {
        ::dlclose(d_->lib);
    }
}

bool MonadoClients::ok() const {
    return d_->root != nullptr;
}

std::optional<std::pair<unsigned, std::string>> MonadoClients::primary() {
    if (!ok() || d_->update(d_->root) != 0) {
        return std::nullopt;
    }
    std::uint32_t n = 0;
    d_->count(d_->root, &n);
    for (std::uint32_t i = 0; i < n; i++) {
        std::uint32_t id = 0, flags = 0;
        const char* nm = nullptr;
        if (d_->idAt(d_->root, i, &id) != 0 || d_->state(d_->root, id, &flags) != 0) {
            continue;
        }
        if ((flags & kPrimary) != 0 && (flags & kOverlay) == 0) {
            d_->name(d_->root, id, &nm);
            return std::pair{static_cast<unsigned>(id), std::string(nm != nullptr ? nm : "")};
        }
    }
    return std::nullopt;
}

bool MonadoClients::blockInputs(unsigned clientId, bool block) {
    return ok() && d_->blocks(d_->root, clientId, block ? kBlockInputs : 0u) == 0;
}

std::optional<long long> MonadoClients::pidOf(unsigned clientId) {
    std::int64_t pid = 0;
    if (!ok() || d_->pid == nullptr || d_->update(d_->root) != 0 || d_->pid(d_->root, clientId, &pid) != 0 || pid <= 0) {
        return std::nullopt;
    }
    return static_cast<long long>(pid);
}

bool MonadoClients::knowsPids() const {
    return ok() && d_->pid != nullptr;
}

} // namespace pibridge::home
