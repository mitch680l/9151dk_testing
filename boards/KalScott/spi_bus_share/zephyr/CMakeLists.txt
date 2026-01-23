# SPI Bus Share Module
# This module adds SPI bus arbitration support

if(CONFIG_SPI_BUS_SHARE)
  get_filename_component(MODULE_ROOT ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)

  zephyr_library_include_directories(${MODULE_ROOT})

  # Bootloader context needs spi_bus_init.c
  if(CONFIG_BOOT_GO_HOOKS)
    message(STATUS "=== Adding SPI bus share module (Bootloader) ===")
    zephyr_library_sources(${MODULE_ROOT}/spi_bus_init.c)
  else()
    message(STATUS "=== Adding SPI bus share module (Application) ===")
  endif()

  zephyr_library_sources(${MODULE_ROOT}/spi_bus_share.c)
  zephyr_library_sources(${MODULE_ROOT}/spi_bus_pin_config.c)


  if(CONFIG_SPI_BUS_SHARE_SHELL)
    zephyr_library_sources(${MODULE_ROOT}/spi_bus_share_shell.c)
    zephyr_compile_definitions(SPI_BUS_SHARE_SHELL)
  endif()

  message(STATUS "=== SPI bus share module added successfully ===")

  if(CONFIG_SPI_BUS_SHARE_BOOTLOADER)
    zephyr_compile_definitions(THIS_IS_BOOTLOADER)
  endif()

  if(CONFIG_SPI_BUS_SHARE_NRF5340)
    zephyr_compile_definitions(THIS_IS_NRF5340)
  endif()

  if(CONFIG_SPI_BUS_SHARE_NRF9151)
    zephyr_compile_definitions(THIS_IS_NRF9151)
  endif()

  if(CONFIG_SPI_BUS_SHARE_DETERM)
    #zephyr_compile_definitions(REPLACE_IDLE_WITH_TRISTATE)
    zephyr_compile_definitions(SPI_BUS_CLAIM_ON_NO_MCU)
  endif()

endif()
