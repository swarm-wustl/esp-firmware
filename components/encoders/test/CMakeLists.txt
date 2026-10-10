# the bank is generic over the counter, so a mock one exercises it on the host
idf_component_register(
    SRCS "encoder_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity encoders
)
