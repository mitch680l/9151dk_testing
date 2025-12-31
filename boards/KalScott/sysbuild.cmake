
#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if(SB_CONFIG_BOARD_KESTREL_NRF9151_NS)
  # Use static partition layout to ensure the partition layout remains
  # unchanged after DFU. This needs to be made globally available
  # because it is used in other CMake files.
  if(SB_CONFIG_KESTREL_STATIC_PARTITIONS_FACTORY)
    set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_LIST_DIR}/kestrel_nrf9151_pm_static.yml CACHE INTERNAL "")
  endif()
endif()

if(SB_CONFIG_BOARD_KESTREL_NRF5340_CPUAPP OR SB_CONFIG_BOARD_KESTREL_NRF5340_CPUAPP_NS)
  if(SB_CONFIG_KESTREL_STATIC_PARTITIONS_FACTORY)
    set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_LIST_DIR}/kestrel_nrf5340_pm_static.yml CACHE INTERNAL "")
  endif()
  if(SB_CONFIG_KESTREL_STATIC_PARTITIONS_NRF53_EXTERNAL_FLASH)
    set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_LIST_DIR}/kestrel_nrf5340_pm_static_ext_flash.yml CACHE INTERNAL "")
  endif()
endif()


# Add SPI bus share module for application (if enabled via Kconfig)
# Note: This is always added to make Kconfig options available
# The actual compilation is controlled by CONFIG_SPI_BUS_SHARE in the module's CMakeLists.txt
set(${DEFAULT_IMAGE}_EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_LIST_DIR}/spi_bus_share CACHE INTERNAL "")

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  # Always add module to MCUboot for image access hooks
  set(mcuboot_EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_LIST_DIR}/spi_bus_share CACHE INTERNAL "")
  set(mcuboot_OVERLAY_CONFIG ${CMAKE_CURRENT_LIST_DIR}/sysbuild/mcuboot.conf CACHE INTERNAL "")
endif()