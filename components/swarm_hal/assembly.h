// Written with Claude
#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#include "drive.h"

// Keep this header bare by forward-declaring chassis_impl
// We don't need its definition for anything that requires Assembly
// (which is a lot of things!)
// Should help keep file sizes/compile times down
namespace Swarm {
template <Drive::Style S, typename Driver, typename... Peripherals>
struct chassis_impl;

// Anything that claims pins must take an Assembly in its ctor
// This way, the only way to construct one of these is through the ::make<T>
// method so the compile-time type checks cannot be circumvented.
// Completely type-safe at comptime by construction.
class Assembly {
  Assembly() = default;

  template <Drive::Style S, typename Driver, typename... Peripherals>
  friend struct chassis_impl;
};
} // namespace Swarm

#endif
