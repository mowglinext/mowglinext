// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_localization/gnss_runtime_state.hpp"

namespace mowgli_localization
{

// Returns a coarse UI-oriented quality score when a backend does not supply
// a richer normalized metric explicitly.
float QualityPercentForFixType(GnssFixType fix_type);
mowgli_interfaces::msg::GnssStatus ToGnssStatusMessage(const GnssRuntimeState& state);

}  // namespace mowgli_localization
