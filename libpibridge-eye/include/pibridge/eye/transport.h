// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/transport.rs (MIT OR Apache-2.0).
// The three USB operations the client needs. Implemented by usbfs::UsbFs (the real chip) and fake::FakeChip (tests).
#pragma once

#include "pibridge/eye/error.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace pibridge::eye {

using Bytes = std::vector<std::uint8_t>;
using ByteView = std::span<const std::uint8_t>;
/// The 8-byte control setup packet (bmRequestType, bRequest, wValue LE, wIndex LE, wLength LE).
using Setup = std::array<std::uint8_t, 8>;
using Duration = std::chrono::nanoseconds;
using Clock = std::chrono::steady_clock;

/// Minimal synchronous USB device interface.
class Transport {
public:
    virtual ~Transport() = default;

    /// Control transfer. `setup` is the 8-byte setup packet; `out` is the data stage of an OUT request (its length
    /// must equal wLength). Returns the data stage of an IN request, empty for OUT requests.
    virtual Result<Bytes> control(const Setup& setup, ByteView out, Duration timeout) = 0;

    /// Bulk OUT transfer of exactly `data` on endpoint `ep`.
    virtual Result<void> bulk_out(std::uint8_t ep, ByteView data, Duration timeout) = 0;

    /// Bulk IN transfer of at most `max_len` bytes; nullopt when nothing arrived before the timeout.
    virtual Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) = 0;
};

} // namespace pibridge::eye
