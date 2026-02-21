# CMakeLists.txt
# ESP-IDF Project CMakeLists for SAR Radar WiFi Bridge
#
# This is the top-level CMakeLists.txt for the ESP-IDF build system.
# It defines the project name and includes the ESP-IDF build framework.
#
# Build instructions:
#   1. Set up ESP-IDF environment:
#      $ . $IDF_PATH/export.sh
#   2. Set target to ESP32-C3:
#      $ idf.py set-target esp32c3
#   3. Configure (optional, defaults in sdkconfig.defaults):
#      $ idf.py menuconfig
#   4. Build:
#      $ idf.py build
#   5. Flash and monitor:
#      $ idf.py -p /dev/ttyUSB0 flash monitor
#
# Author: SAR Radar Project
# Date:   2026-02-21

cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project name
project(sar_radar_wifi_bridge)
