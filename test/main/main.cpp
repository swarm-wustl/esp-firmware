#include "unity.h"

TEST_CASE("sanity check", "[basic]") {
    TEST_ASSERT_EQUAL(1, 1);
}

extern "C" void app_main(void) {
    unity_run_all_tests();
    unity_run_tests_by_tag("[dwm]", false);
}