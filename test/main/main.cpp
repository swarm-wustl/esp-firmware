#include "unity.h"
#include <cstdlib>

TEST_CASE("sanity check", "[basic]") { TEST_ASSERT_EQUAL(1, 1); }

extern "C" void app_main(void) {
  unity_run_all_tests();

#if CONFIG_IDF_TARGET_LINUX
  // the linux scheduler never returns, so exit with a status CI can read
  exit(Unity.TestFailures == 0 ? 0 : 1);
#endif
}
