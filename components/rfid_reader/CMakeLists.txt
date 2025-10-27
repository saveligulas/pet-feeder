idf_component_register(
    SRCS "src/rfid_reader.c" "src/i2c_setup.c"
    INCLUDE_DIRS "include"
    REQUIRES garag__esp-idf-pn532
)
