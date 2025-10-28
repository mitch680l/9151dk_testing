cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


project(9151_testing)

target_sources(app PRIVATE src/main.c)
#target_sources(app PRIVATE src/spi_thread.c)
#target_soruces(app PRIVATE src/lte_helper.c)
#target_sources(app PRIVATE src/fota.c)