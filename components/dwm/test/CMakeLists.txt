# host-runnable unit tests (pure value types + mock SPI); build on every target
set(srcs "dwm_data_test.cpp" "mock_reg_view_test.cpp")
set(reqs unity dwm)

# hardware integration tests need real ESP32 peripherals, so skip on the host
if(NOT "${IDF_TARGET}" STREQUAL "linux")
    list(APPEND srcs "reg_view.cpp" "test.cpp")
    list(APPEND reqs esp32)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${reqs}
)
