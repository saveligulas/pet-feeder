idf_component_register(
    SRCS "src/pet-feeder.c"
    REQUIRES rfid_reader pi_communicator freertos esp_system
)
