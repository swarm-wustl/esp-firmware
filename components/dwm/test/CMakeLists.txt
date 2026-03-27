file(GLOB_RECURSE SOURCES "*.cpp")

# TODO: probably shouldn't rely on esp32, just mocks... but not sure
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES unity dwm esp32
)

