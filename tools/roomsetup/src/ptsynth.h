// SPDX-License-Identifier: MIT
// The passthrough without a headset (tools/xrkit ptsynth.h), under the names the room setup's tests use.
#pragma once

#include "pibridge/xrkit/ptsynth.h"

namespace pibridge::roomsetup {

using xrkit::ptsynth::geomPose;
using xrkit::ptsynth::headPose;
using xrkit::ptsynth::referenceRig;
using xrkit::ptsynth::roomShape;
using xrkit::ptsynth::synthFrame;

} // namespace pibridge::roomsetup
