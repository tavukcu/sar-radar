# CMakeLists.txt for the main component
#
# ESP-IDF uses a component-based build system. The 'main' component
# is the entry point and contains the application source files.
#
# Additional components (e.g., custom drivers, libraries) can be
# added as separate directories under the 'components/' folder.

idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_ota_ops
        lwip
)
