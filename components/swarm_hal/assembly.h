// Written with Claude
#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#include "resources.h"

namespace Swarm {
template <HAL::Claiming... Peripherals> struct system_impl;

// peripherals that claim hardware take one of these to be constructed, and
// only a system can make one -- so nothing that owns a pin can exist outside a
// config that declared it. A device that builds a sub-device passes its own
// token down
class Assembly {
  Assembly() = default;

  template <HAL::Claiming... Peripherals> friend struct system_impl;
};
} // namespace Swarm

#endif
