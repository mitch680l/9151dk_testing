#ifndef SPI_BUS_SHARE_CONFIG_H
#define SPI_BUS_SHARE_CONFIG_H

/**
 * @file spi_bus_share_config.h
 * @brief Configuration definitions for SPI bus sharing module
 *
 * This file contains all platform-specific pin definitions and timing constants
 * used by the SPI bus sharing module.
 */

/* Feature flags */
#define NO_CS_CHANGE
// #define ENABLE_CS_MIRROR

#ifdef THIS_IS_NRF5340
	/* nRF5340 configuration */
	#define SPI_SCK_PIN   17
	#define SPI_MOSI_PIN  13
	#define SPI_MISO_PIN  14
	#define SPI_CS_PIN    18

	#define SPI_CS1_PIN   27
	#define SPI_CS2_PIN   15

	#define BUS_REQUEST_PIN  2
	#define BUS_GRANT_PIN    3

	#define INITIAL_FSM_STATE     FSM_IDLE_OWNER
	#define INITIAL_OWNER         SPI_BUS_OWNER_THIS_MCU
	#define SPI_BUS_TYPE          "QSPI"
#else
	/* nRF9151 configuration */
	#define SPI_SCK_PIN   2
	#define SPI_MOSI_PIN  3
	#define SPI_MISO_PIN  1
	#define SPI_CS_PIN    0    

	#define SPI_CS1_PIN   27
	#define SPI_CS2_PIN   15

	#define BUS_REQUEST_PIN  19
	#define BUS_GRANT_PIN    18

	#define INITIAL_FSM_STATE     FSM_IDLE_NOT_OWNER
	#define INITIAL_OWNER         SPI_BUS_OWNER_OTHER_MCU
	#define SPI_BUS_TYPE          "SPI"

	#ifdef ENABLE_CS_MIRROR
		#define CS_MIRROR_NRF5340_SOURCE_PIN  15  
		#define CS_MIRROR_TARGET_PIN          0   
	#endif
#endif

#define BUS_REQUEST_TIMEOUT_MS     120000  
#define BUS_QUICK_GRANT_TIMEOUT_MS 500   
#define BUS_MISSING_MCU_TIMEOUT_MS 2000   
#define BUS_LEASE_TIME_MS          5000    
#define BUS_MAX_HOG_TIME_MS        30000   
#define BUS_RELEASE_TIMEOUT_MS     500    

#define BOOT_DELAY_MS            5000
#endif /* SPI_BUS_SHARE_CONFIG_H */
