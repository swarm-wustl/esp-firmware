// Written with Claude
#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#include "drive.h"

namespace Swarm {
template <Drive::Style S, typename Driver, typename... Peripherals>
struct chassis_impl;

// peripherals that claim pins take one of these to be constructed, and only a
// chassis can make one -- so nothing that owns hardware can exist outside a
// config that declared it
class Assembly {
  Assembly() = default;

  template <Drive::Style S, typename Driver, typename... Peripherals>
  friend struct chassis_impl;
};
} // namespace Swarm

#endif
