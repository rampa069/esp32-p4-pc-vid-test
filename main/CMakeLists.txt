idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_timer nvs_flash driver esp_hw_support esp_mm
)
